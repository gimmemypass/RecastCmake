#include "Shell.h"

float random_float();

Shell::Shell(char* logpath) {
	ctx = new Context(logpath);
	ctx->enableLog(true);
}
Shell::~Shell() {};
bool Shell::handleBuild(rcConfig* cfg, float* verts, int nverts, int* tris, int ntris)
{
	// Calc boundaries
	float bmin[3];
	float bmax[3];
	rcCalcBounds(verts, nverts / 3, bmin, bmax);

	// Set the area where the navigation will be build.
	// Here the bounds of the input mesh are used, but the
	// area could be specified by an user defined box, etc.
	rcVcopy(cfg->bmin, bmin);
	rcVcopy(cfg->bmax, bmax);
	rcCalcGridSize(cfg->bmin, cfg->bmax, cfg->cs, &cfg->width, &cfg->height);

	// Reset build times gathering.
	ctx->resetTimers();

	// Start the build process.	
	ctx->startTimer(RC_TIMER_TOTAL);

	ctx->log(RC_LOG_PROGRESS, "Building navigation:");
	ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", cfg->width, cfg->height);
	ctx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts / 1000.0f, ntris / 1000.0f);

	//
	// Step 2. Rasterize input polygon soup.
	//

	// Allocate voxel heightfield where we rasterize our input data to.
	rcHeightfield* solid = rcAllocHeightfield();
	if (!solid)
	{
		return false;
	}
	if (!rcCreateHeightfield(ctx, *solid, cfg->width, cfg->height, cfg->bmin, cfg->bmax, cfg->cs, cfg->ch))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return false;
	}

	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	unsigned char* triareas = new unsigned char[ntris];
	if (!triareas)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
		return false;
	}

	// Find triangles which are walkable based on their slope and rasterize them.
	// If your input data is multiple meshes, you can transform them here, calculate
	// the are type for each of the meshes and rasterize them.
	memset(triareas, 0, ntris * sizeof(unsigned char));
	rcMarkWalkableTriangles(ctx, cfg->walkableSlopeAngle, verts, nverts, tris, ntris, triareas);
	rcRasterizeTriangles(ctx, verts, nverts, tris, triareas, ntris, *solid, cfg->walkableClimb);

	//if (!m_keepInterResults)
	{
		delete[] triareas;
		triareas = 0;
	}

	//
	// Step 3. Filter walkables surfaces.
	//

	// Once all geoemtry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	rcFilterLowHangingWalkableObstacles(ctx, cfg->walkableClimb, *solid);
	rcFilterLedgeSpans(ctx, cfg->walkableHeight, cfg->walkableClimb, *solid);
	rcFilterWalkableLowHeightSpans(ctx, cfg->walkableHeight, *solid);

	//
	// Step 4. Partition walkable surface to simple regions.
	//

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	rcCompactHeightfield* chf = rcAllocCompactHeightfield();
	if (!chf)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return false;
	}
	if (!rcBuildCompactHeightfield(ctx, cfg->walkableHeight, cfg->walkableClimb, *solid, *chf))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return false;
	}

	//if (!m_keepInterResults)
	{
		rcFreeHeightField(solid);
		solid = 0;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(ctx, cfg->walkableRadius, *chf))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return false;
	}

	// (Optional) Mark areas.
	//const ConvexVolume* vols = m_geom->getConvexVolumes();
	//for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
	//rcMarkConvexPolyArea(&ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *chf);

	if (monotonePartitioning)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(ctx, *chf, 0, cfg->minRegionArea, cfg->mergeRegionArea))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
			return false;
		}
	}
	else
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(ctx, *chf))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return false;
		}

		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(ctx, *chf, 0, cfg->minRegionArea, cfg->mergeRegionArea))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
			return false;
		}
	}

	//
	// Step 5. Trace and simplify region contours.
	//

	// Create contours.
	rcContourSet* cset = rcAllocContourSet();
	if (!cset)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return false;
	}
	if (!rcBuildContours(ctx, *chf, cfg->maxSimplificationError, cfg->maxEdgeLen, *cset))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return false;
	}

	//
	// Step 6. Build polygons mesh from contours.
	//

	// Build polygon navmesh from the contours.
	pmesh = rcAllocPolyMesh();
	if (!pmesh)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return false;
	}
	if (!rcBuildPolyMesh(ctx, *cset, cfg->maxVertsPerPoly, *pmesh))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return false;
	}

	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//

	dmesh = rcAllocPolyMeshDetail();
	if (!dmesh)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return false;
	}

	if (!rcBuildPolyMeshDetail(ctx, *pmesh, *chf, cfg->detailSampleDist, cfg->detailSampleMaxError, *dmesh))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		return false;
	}

	//if (!m_keepInterResults)
	{
		rcFreeCompactHeightfield(chf);
		chf = 0;
		rcFreeContourSet(cset);
		cset = 0;
	}

	// At this point the navigation mesh data is ready, you can access it from m_pmesh.
	// See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.

	//
	// (Optional) Step 8. Create Detour data from Recast poly mesh.
	//
	// ...

	ctx->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats.
	//duLogBuildTimes(*&ctx, ctx.getAccumulatedTime(RC_TIMER_TOTAL));
	ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", pmesh->nverts, pmesh->npolys);

	//m_totalBuildTimeMs = ctx.getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;
	return true;
}

rcPolyMesh* Shell::getPolyMesh()
{
	return pmesh;
}

rcPolyMeshDetail* Shell::getPolyMeshDetail()
{
	return dmesh;
}

bool Shell::createNavmesh(rcConfig* cfg, rcPolyMesh* pmesh, rcPolyMeshDetail* dmesh, unsigned char*& navData, int& dataSize)
{
	dtNavMeshCreateParams* createParams = new dtNavMeshCreateParams();
	memset(createParams, 0, sizeof(dtNavMeshCreateParams));

	createParams->verts = pmesh->verts;
	createParams->vertCount = pmesh->nverts;
	createParams->polys = pmesh->polys;
	createParams->polyFlags = pmesh->flags;
	createParams->polyAreas = pmesh->areas;
	createParams->polyCount = pmesh->npolys;
	createParams->nvp = pmesh->nvp;

	createParams->detailMeshes = dmesh->meshes;
	createParams->detailVerts = dmesh->verts;
	createParams->detailVertsCount = dmesh->nverts;
	createParams->detailTris = dmesh->tris;
	createParams->detailTriCount = dmesh->ntris;

	createParams->walkableHeight = cfg->walkableHeight;
	createParams->walkableRadius = cfg->walkableRadius;
	createParams->walkableClimb = cfg->walkableClimb;
	createParams->cs = cfg->cs;
	createParams->ch = cfg->ch;

	createParams->buildBvTree = true;

	if (dtCreateNavMeshData(createParams, &navData, &dataSize))
	{
		dtNavMesh* navmesh = dtAllocNavMesh();
		navmesh->init(navData, dataSize, DT_TILE_FREE_DATA);

		return true;
	}

	return false;
}

// InputGeometry* Shell::getInputGeometryFromObj(rcMeshLoaderObj* meshLoader, const char* filePathArray, int filePathLen)
// {
// 	int i = 0;
// 	std::string filePath = "";
// 	for (i = 0; i < filePathLen; i++)
// 	{
// 		filePath = filePath + filePathArray[i];
// 	}
// 	meshLoader->load(filePath);
// 	InputGeometry* geom = new InputGeometry();
// 	geom->ntris = meshLoader->getTriCount();
// 	geom->nverts = meshLoader->getVertCount();
// 	int* tris = new int[geom->ntris * 3];
// 	for (i = 0; i < geom->ntris * 3; i++)
// 		tris[i] = meshLoader->getTris()[i];

// 	float* verts = new float[geom->nverts];
// 	for (i = 0; i < geom->nverts; i++)
// 		verts[i] = meshLoader->getVerts()[i];

// 	geom->tris = tris;
// 	geom->verts = verts;
// 	return geom;
// }

// rcMeshLoaderObj* Shell::allocMeshLoader()
// {
// 	return new rcMeshLoaderObj;
// }

int Shell::rasterizeTileLayers(Context* ctx, TileCacheHolder* holder,
	const int tx, const int ty,
	const rcConfig* cfg,
	TileCacheData* tiles,
	const int maxTiles)
{
	FastLZCompressor comp;
	RasterizationContext rc;

	const float* verts = holder->geom->verts;
	const int nverts = holder->geom->nverts;
	const rcChunkyTriMesh* chunkyMesh = holder->chunkyMesh;

	// Tile bounds.
	const float tcs = cfg->tileSize * cfg->cs;

	rcConfig tcfg;
	memcpy(&tcfg, cfg, sizeof(tcfg));

	tcfg.bmin[0] = cfg->bmin[0] + tx * tcs;
	tcfg.bmin[1] = cfg->bmin[1];
	tcfg.bmin[2] = cfg->bmin[2] + ty * tcs;
	tcfg.bmax[0] = cfg->bmin[0] + (tx + 1) * tcs;
	tcfg.bmax[1] = cfg->bmax[1];
	tcfg.bmax[2] = cfg->bmin[2] + (ty + 1) * tcs;
	tcfg.bmin[0] -= tcfg.borderSize * tcfg.cs;
	tcfg.bmin[2] -= tcfg.borderSize * tcfg.cs;
	tcfg.bmax[0] += tcfg.borderSize * tcfg.cs;
	tcfg.bmax[2] += tcfg.borderSize * tcfg.cs;

	// Allocate voxel heightfield where we rasterize our input data to.
	rc.solid = rcAllocHeightfield();
	if (!rc.solid)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return 0;
	}
	if (!rcCreateHeightfield(ctx, *rc.solid, tcfg.width, tcfg.height, tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return 0;
	}

	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	rc.triareas = new unsigned char[chunkyMesh->maxTrisPerChunk];
	if (!rc.triareas)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", chunkyMesh->maxTrisPerChunk);
		return 0;
	}

	float tbmin[2], tbmax[2];
	tbmin[0] = tcfg.bmin[0];
	tbmin[1] = tcfg.bmin[2];
	tbmax[0] = tcfg.bmax[0];
	tbmax[1] = tcfg.bmax[2];
	int cid[512];// TODO: Make grow when returning too many items.
	const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
	if (!ncid)
	{
		return 0; // empty
	}

	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
		const int* tris = &chunkyMesh->tris[node.i * 3];
		const int ntris = node.n;

		memset(rc.triareas, 0, ntris * sizeof(unsigned char));
		rcMarkWalkableTriangles(ctx, tcfg.walkableSlopeAngle,
			verts, nverts, tris, ntris, rc.triareas);

		rcRasterizeTriangles(ctx, verts, nverts, tris, rc.triareas, ntris, *rc.solid, tcfg.walkableClimb);
	}

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	rcFilterLowHangingWalkableObstacles(ctx, tcfg.walkableClimb, *rc.solid);
	rcFilterLedgeSpans(ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid);
	rcFilterWalkableLowHeightSpans(ctx, tcfg.walkableHeight, *rc.solid);


	rc.chf = rcAllocCompactHeightfield();
	if (!rc.chf)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return 0;
	}
	if (!rcBuildCompactHeightfield(ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid, *rc.chf))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return 0;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(ctx, tcfg.walkableRadius, *rc.chf))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return 0;
	}

	// (Optional) Mark areas.
	for (int i = 0; i < numConvexVolumes; ++i)
	{
		rcMarkConvexPolyArea(ctx, convexVolumes[i].verts, convexVolumes[i].nverts,
			convexVolumes[i].hmin, convexVolumes[i].hmax,
			(unsigned char)convexVolumes[i].area, *rc.chf);
	}

	rc.lset = rcAllocHeightfieldLayerSet();
	if (!rc.lset)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'lset'.");
		return 0;
	}
	if (!rcBuildHeightfieldLayers(ctx, *rc.chf, tcfg.borderSize, tcfg.walkableHeight, *rc.lset))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build heighfield layers.");
		return 0;
	}

	rc.ntiles = 0;
	for (int i = 0; i < rcMin(rc.lset->nlayers, MAX_LAYERS); ++i)
	{
		TileCacheData* tile = &rc.tiles[rc.ntiles++];
		const rcHeightfieldLayer* layer = &rc.lset->layers[i];

		// Store header
		dtTileCacheLayerHeader header;
		header.magic = DT_TILECACHE_MAGIC;
		header.version = DT_TILECACHE_VERSION;

		// Tile layer location in the navmesh.
		header.tx = tx;
		header.ty = ty;
		header.tlayer = i;
		dtVcopy(header.bmin, layer->bmin);
		dtVcopy(header.bmax, layer->bmax);

		// Tile info.
		header.width = (unsigned char)layer->width;
		header.height = (unsigned char)layer->height;
		header.minx = (unsigned char)layer->minx;
		header.maxx = (unsigned char)layer->maxx;
		header.miny = (unsigned char)layer->miny;
		header.maxy = (unsigned char)layer->maxy;
		header.hmin = (unsigned short)layer->hmin;
		header.hmax = (unsigned short)layer->hmax;

		dtStatus status = dtBuildTileCacheLayer(&comp, &header, layer->heights, layer->areas, layer->cons,
			&tile->data, &tile->dataSize);
		if (dtStatusFailed(status))
		{
			return 0;
		}
	}

	// Transfer ownsership of tile data from build context to the caller.
	int n = 0;
	for (int i = 0; i < rcMin(rc.ntiles, maxTiles); ++i)
	{
		tiles[n++] = rc.tiles[i];
		rc.tiles[i].data = 0;
		rc.tiles[i].dataSize = 0;
	}

	return n;
}


bool Shell::handleTileCacheBuild(rcConfig* cfg, ExtendedConfig* ecfg, InputGeometry* geom,
	dtTileCache*& tileCache, dtNavMesh*& navMesh, dtNavMeshQuery*& navQuery)
{
	TileCacheHolder* holder = new TileCacheHolder();

	holder->cfg = cfg;
	holder->ecfg = ecfg;
	holder->geom = geom;

	navQuery = dtAllocNavMeshQuery();

	float bmin[3];
	float bmax[3];
	rcCalcBounds(geom->verts, geom->nverts / 3, bmin, bmax);

	holder->chunkyMesh = new rcChunkyTriMesh;
	if (!holder->chunkyMesh)
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'm_chunkyMesh'.");
		return false;
	}
	if (!rcCreateChunkyTriMesh(geom->verts, geom->tris, geom->ntris, 256, holder->chunkyMesh))
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Failed to build chunky mesh.");
		return false;
	}

	//m_tmproc->init(m_geom);

	// Init cache
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, cfg->cs, &gw, &gh);
	const int ts = cfg->tileSize;
	const int tw = (gw + ts - 1) / ts;
	const int th = (gh + ts - 1) / ts;

	int tileBits = rcMin((int)dtIlog2(dtNextPow2(tw * th * EXPECTED_LAYERS_PER_TILE)), 14);
	if (tileBits > 14) tileBits = 14;
	int polyBits = 22 - tileBits;
	int maxTiles = 1 << tileBits;
	int maxPolysPerTile = 1 << polyBits;

	// Generation params.
	cfg->borderSize = cfg->walkableRadius + 3; // Reserve enough padding.
	cfg->width = cfg->tileSize + cfg->borderSize * 2;
	cfg->height = cfg->tileSize + cfg->borderSize * 2;

	rcVcopy(cfg->bmin, bmin);
	rcVcopy(cfg->bmax, bmax);

	// Tile cache params.
	dtTileCacheParams tcparams;
	memset(&tcparams, 0, sizeof(tcparams));
	rcVcopy(tcparams.orig, bmin);
	tcparams.cs = cfg->cs;
	tcparams.ch = cfg->ch;
	tcparams.width = cfg->tileSize;
	tcparams.height = cfg->tileSize;
	tcparams.walkableHeight = ecfg->AgentHeight;
	tcparams.walkableRadius = ecfg->AgentRadius;
	tcparams.walkableClimb = ecfg->AgentMaxClimb;
	tcparams.maxSimplificationError = cfg->maxSimplificationError;
	tcparams.maxTiles = tw * th * EXPECTED_LAYERS_PER_TILE;
	tcparams.maxObstacles = ecfg->MaxObstacles;

	tileCache = dtAllocTileCache();
	if (!tileCache)
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate tile cache.");
		return false;
	}

	dtStatus status = tileCache->init(&tcparams, allocator, compressor, processor);
	if (dtStatusFailed(status))
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init tile cache.");
		return false;
	}

	navMesh = dtAllocNavMesh();
	if (!navMesh)
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return false;
	}

	dtNavMeshParams params;
	memset(&params, 0, sizeof(params));
	rcVcopy(params.orig, bmin);
	params.tileWidth = cfg->tileSize * cfg->cs;
	params.tileHeight = cfg->tileSize * cfg->cs;
	params.maxTiles = maxTiles;
	params.maxPolys = maxPolysPerTile;

	status = navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
		return false;
	}

	status = navQuery->init(navMesh, 2048);
	if (dtStatusFailed(status))
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
		return false;
	}

	// Preprocess tiles.

	ctx->resetTimers();

	int cacheLayerCount = 0;
	int cacheCompressedSize = 0;
	int cacheRawSize = 0;

	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			TileCacheData tiles[MAX_LAYERS];
			memset(tiles, 0, sizeof(tiles));
			int ntiles = rasterizeTileLayers(ctx, holder, x, y, cfg, tiles, MAX_LAYERS);

			for (int i = 0; i < ntiles; ++i)
			{
				TileCacheData* tile = &tiles[i];
				status = tileCache->addTile(tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);
				if (dtStatusFailed(status))
				{
					dtFree(tile->data);
					tile->data = 0;
					continue;
				}

				cacheLayerCount++;
				cacheCompressedSize += tile->dataSize;
				//cacheRawSize += calcLayerBufferSize(tcparams.width, tcparams.height);
			}
		}
	}

	// Build initial meshes
	ctx->startTimer(RC_TIMER_TOTAL);
	for (int y = 0; y < th; ++y)
		for (int x = 0; x < tw; ++x)
			tileCache->buildNavMeshTilesAt(x, y, navMesh);
	ctx->stopTimer(RC_TIMER_TOTAL);
	return true;
}

void Shell::addFlag(unsigned short area, unsigned short cost)
{
	// short index = 0;
	// short t_area = area;
	// while (t_area != 1) 
	// {
	// 	t_area /= 2;
	// 	index++;
	// }
	// if (flags[index] != 0)
	// 	return;
	flags[numFlags] = area;
	costs[numFlags] = cost;
	++numFlags;
}

void Shell::addConvexVolume(float* verts, int nverts, float hmax, float hmin, int area)
{
	memcpy(convexVolumes[numConvexVolumes].verts, verts, nverts * 3);
	convexVolumes[numConvexVolumes].nverts = nverts;
	convexVolumes[numConvexVolumes].hmax = hmax;
	convexVolumes[numConvexVolumes].hmin = hmin;
	convexVolumes[numConvexVolumes].area = area;
	++numConvexVolumes;
}

void Shell::getTileCacheHeaders(TileCacheSetHeader& header, TileCacheTileHeader*& tilesHeader, dtTileCache* tileCache, dtNavMesh* navMesh)
{
	// Store header.
	header.magic = TILECACHESET_MAGIC;
	header.version = TILECACHESET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < tileCache->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = tileCache->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.cacheParams, tileCache->getParams(), sizeof(dtTileCacheParams));
	memcpy(&header.meshParams, navMesh->getParams(), sizeof(dtNavMeshParams));

	// Allocate memory
	int n = 0;
	tilesHeader = new TileCacheTileHeader[header.numTiles];

	// Store tiles.
	for (int i = 0; i < tileCache->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = tileCache->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		tilesHeader[n].tileRef = tileCache->getTileRef(tile);
		tilesHeader[n].dataSize = tile->dataSize;

		std::string tileDebug;
		tileDebug.append(std::to_string(i));
		tileDebug.append(" ");
		tileDebug.append(std::to_string(tile->header->tx));
		tileDebug.append(" ");
		tileDebug.append(std::to_string(tile->header->ty));
		DebugCPP::Log(tileDebug);

		++n;
	}
}

bool Shell::loadFromTileCacheHeaders(TileCacheSetHeader* header, TileCacheTileHeader* tilesHeader, unsigned char* data, dtTileCache*& tileCache, dtNavMesh*& navMesh, dtNavMeshQuery*& navQuery)
{
	if (header->magic != TILECACHESET_MAGIC)
	{
		ctx->log(RC_LOG_ERROR, "FAILED MAGIC");
		return false;
	}
	if (header->version != TILECACHESET_VERSION)
	{
		ctx->log(RC_LOG_ERROR, "FAILED VERSION");
		return false;
	}

	navMesh = dtAllocNavMesh();
	if (!navMesh)
	{
		ctx->log(RC_LOG_ERROR, "FAILED dtAllocNavMesh");
		return false;
	}

	dtStatus status = navMesh->init(&header->meshParams);
	if (dtStatusFailed(status))
	{
		ctx->log(RC_LOG_ERROR, "FAILED navMesh->init");
		return false;
	}

	tileCache = dtAllocTileCache();
	if (!tileCache)
	{
		ctx->log(RC_LOG_ERROR, "FAILED dtAllocTileCache");
		return false;
	}
	status = tileCache->init(&header->cacheParams, allocator, compressor, processor);
	if (dtStatusFailed(status))
	{
		ctx->log(RC_LOG_ERROR, "FAILED tileCache->init");
		return false;
	}

	// Read tiles.
	int n = 0;
	int start = 0;
	for (int i = 0; i < header->numTiles; ++i)
	{
		TileCacheTileHeader& tileHeader = tilesHeader[n++];
		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		dtCompressedTileRef tile = 0;
		tileCache->addTile(data + start, tileHeader.dataSize, DT_COMPRESSEDTILE_FREE_DATA, &tile);
		start += tileHeader.dataSize;

		dtStatus status = DT_FAILURE;
		if (tile)
			status = tileCache->buildNavMeshTile(tile, navMesh);

		if (dtStatusFailed(status))
		{
			ctx->log(RC_LOG_ERROR, "FAILED BUILDING TILE %d [%x] [[%lld]]", i, status, status);
		}
	}

	navQuery = dtAllocNavMeshQuery();
	status = navQuery->init(navMesh, 2048);
	if (dtStatusFailed(status))
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
		return false;
	}

	DebugCPP::Log("loadFromTileCacheHeaders");
	for (int i = 0; i < tileCache->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = tileCache->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		std::stringstream addressDebug;
		addressDebug << tile->header;

		std::string tileDebug;
		tileDebug.append(std::to_string(i));
		tileDebug.append(" ");
		tileDebug.append(addressDebug.str());
		tileDebug.append(" ");
		tileDebug.append(std::to_string(tile->header->tx));
		tileDebug.append(" ");
		tileDebug.append(std::to_string(tile->header->ty));
		DebugCPP::Log(tileDebug);
	}

	//const dtCompressedTile* tile = tileCache->getTile(0);
	//while (tile) {
	//	std::string tileDebug;
	//	tileDebug.append(std::to_string(tile->salt));
	//	tileDebug.append(" ");
	//	tileDebug.append(std::to_string(tile->header->tx));
	//	tileDebug.append(" ");
	//	tileDebug.append(std::to_string(tile->header->ty));
	//	DebugCPP::Log(tileDebug);
	//	tile = tile->next;
	//}

	return true;
}

dtCompressedTile* Shell::getTileCacheTile(dtTileCache* tileCache, int i)
{
	return (dtCompressedTile*)tileCache->getTile(i);
}

dtMeshTile* Shell::getTile(dtNavMesh* navmesh, int i)
{
	return (dtMeshTile*)navmesh->getTileIdx(i);
}

dtObstacleRef Shell::addObstacle(dtTileCache* tileCache, float* pos, float* verts, int nverts, float height, int* result)
{
	dtObstacleRef ref;
	*result = tileCache->addObstacle(pos, verts, nverts, height, &ref);
	DebugCPP::Log("Obstacle added with code");
	DebugCPP::Log(*result);
	return ref;
}

dtObstacleRef Shell::addObstacle(dtTileCache* tileCache, float* pos, float radius, float height, int* result)
{
	dtObstacleRef ref;
	*result = tileCache->addObstacle(pos, radius, height, &ref);
	DebugCPP::Log("Obstacle added with code");
	DebugCPP::Log(*result);
	return ref;
}

unsigned char Shell::getObstacleState(dtTileCache* tileCache, dtObstacleRef ref)
{
	const dtTileCacheObstacle* ob = tileCache->getObstacleByRef(ref);
	return ob->state;
}

void Shell::removeObstacle(dtTileCache* tileCache, dtObstacleRef ref)
{
	tileCache->removeObstacle(ref);
}

//float* Shell::getObstacles(dtTileCache* tc, int& nobstacles)
//{
//	nobstacles = tc->getObstacleCount();
//
//	if (nobstacles > 0)
//	{
//		float* vertices = new float[nobstacles * 6];
//
//		// Draw obstacles
//		for (int i = 0; i < nobstacles; ++i)
//		{
//			const dtTileCacheObstacle* ob = tc->getObstacle(i);
//			if (ob->state == DT_OBSTACLE_EMPTY) continue;
//			tc->getObstacleBounds(ob, &vertices[i * 6], &vertices[i * 6 + 3]);
//
//			float bmin[3], bmax[3];
//			tc->getObstacleBounds(ob, bmin, bmax);
//		}
//
//		return vertices;
//	}
//
//	return NULL;
//}

dtCrowd* Shell::createCrowd(int maxAgents, float maxRadius, dtNavMesh* navmesh)
{
	dtCrowd* crowd = dtAllocCrowd();
	crowd->init(maxAgents, maxRadius, navmesh);

	dtObstacleAvoidanceParams params;
	// Use mostly default settings, copy from dtCrowd.
	memcpy(&params, crowd->getObstacleAvoidanceParams(0), sizeof(dtObstacleAvoidanceParams));

	// Low (11)
	params.velBias = 0.5f;
	params.adaptiveDivs = 5;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 1;
	crowd->setObstacleAvoidanceParams(0, &params);

	// Medium (22)
	params.velBias = 0.5f;
	params.adaptiveDivs = 5;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 2;
	crowd->setObstacleAvoidanceParams(1, &params);

	// Good (45)
	params.velBias = 0.5f;
	params.adaptiveDivs = 7;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 3;
	crowd->setObstacleAvoidanceParams(2, &params);

	// High (66)
	params.velBias = 0.5f;
	params.adaptiveDivs = 7;
	params.adaptiveRings = 3;
	params.adaptiveDepth = 3;

	crowd->setObstacleAvoidanceParams(3, &params);
	return crowd;
}

void Shell::setFilter(dtCrowd* crowd, int filter, unsigned short include, unsigned short exclude)
{
	crowd->getEditableFilter(filter)->setIncludeFlags(include);
	crowd->getEditableFilter(filter)->setExcludeFlags(exclude);
}

int Shell::addAgent(dtCrowd* crowd, float* p, dtCrowdAgentParams* ap)
{
	// Randomize now, because yes!
	//srand(time(NULL));

	return crowd->addAgent(p, ap);
}

dtCrowdAgent* Shell::getAgent(dtCrowd* crowd, int idx)
{
	return (dtCrowdAgent*)crowd->getAgent(idx);
}

void Shell::updateAgent(dtCrowd* crowd, int idx, dtCrowdAgentParams* ap)
{
	crowd->updateAgentParameters(idx, ap);
}

void Shell::removeAgent(dtCrowd* crowd, int idx)
{
	crowd->removeAgent(idx);
}

void Shell::calcVel(float* vel, const float* pos, const float* tgt, const float speed)
{
	dtVsub(vel, tgt, pos);
	vel[1] = 0.0;
	dtVnormalize(vel);
	dtVscale(vel, vel, speed);
}

void Shell::setMoveTarget(dtNavMeshQuery* navquery, dtCrowd* crowd, int idx, float* p, bool adjust, int filterIndex)
{
	// Find nearest point on navmesh and set move request to that location.
	const dtQueryFilter* filter = crowd->getFilter(filterIndex);
	const float* ext = crowd->getQueryExtents();

	if (adjust)
	{
		float vel[3];
		// Request velocity
		if (idx != -1)
		{
			const dtCrowdAgent* ag = crowd->getAgent(idx);
			if (ag && ag->active)
			{
				calcVel(vel, ag->npos, p, ag->params.maxSpeed);
				crowd->requestMoveVelocity(idx, vel);
			}
		}
		else
		{
			for (int i = 0; i < crowd->getAgentCount(); ++i)
			{
				const dtCrowdAgent* ag = crowd->getAgent(i);
				if (!ag->active) continue;
				calcVel(vel, ag->npos, p, ag->params.maxSpeed);
				crowd->requestMoveVelocity(i, vel);
			}
		}
	}
	else
	{
		dtPolyRef targetRef;
		float targetPos[3] = { 0, 0, 0 };
		dtStatus status = navquery->findNearestPoly(p, ext, filter, &targetRef, targetPos);

		if (idx != -1)
		{
			const dtCrowdAgent* ag = crowd->getAgent(idx);
			if (ag && ag->active)
			{
				crowd->requestMoveTarget(idx, targetRef, targetPos);
			}
		}
		else
		{
			for (int i = 0; i < crowd->getAgentCount(); ++i)
			{
				const dtCrowdAgent* ag = crowd->getAgent(i);
				if (!ag->active) continue;
				crowd->requestMoveTarget(i, targetRef, targetPos);
			}
		}
	}
}

void Shell::resetPath(dtCrowd* crowd, int idx)
{
	crowd->resetMoveTarget(idx);
}

void Shell::updateTick(dtTileCache* tileCache, dtNavMesh* nav, dtCrowd* crowd, float dt, float* positions, float* velocity, unsigned char* state, unsigned char* targetState, bool* partial, int& nagents)
{
	if (!nav || !crowd) return;

	tileCache->update(dt, nav);
	crowd->update(dt, NULL);

	// Update agent trails
	nagents = crowd->getAgentCount();
	for (int i = 0; i < nagents; ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);

		positions[i * 3 + 0] = ag->npos[0];
		positions[i * 3 + 1] = ag->npos[1];
		positions[i * 3 + 2] = ag->npos[2];

		velocity[i * 3 + 0] = ag->vel[0];
		velocity[i * 3 + 1] = ag->vel[1];
		velocity[i * 3 + 2] = ag->vel[2];

		state[i] = ag->state;
		targetState[i] = ag->targetState;
		partial[i] = ag->partial;
	}
}

bool Shell::isPointValid(dtCrowd* crowd, float* targetPoint)
{
	const dtNavMeshQuery* navQuery = crowd->getNavMeshQuery();
	const dtQueryFilter* filter = crowd->getFilter(0);
	const float* ext = crowd->getQueryExtents();

	dtPolyRef ref;
	float point[3];

	navQuery->findNearestPoly(targetPoint, ext, filter, &ref, point);

	//return navQuery->isValidPolyRef(ref, filter);
	float heigth;
	dtStatus status = navQuery->getPolyHeight(ref, targetPoint, &heigth);
	return dtStatusSucceed(status);
}

bool Shell::randomPoint(dtCrowd* crowd, float* targetPoint)
{
	const dtNavMeshQuery* navQuery = crowd->getNavMeshQuery();
	const dtQueryFilter* filter = crowd->getFilter(0);
	dtPolyRef targetRef;

	dtStatus status = navQuery->findRandomPoint(filter, random_float, &targetRef, targetPoint);
	return dtStatusSucceed(status);
}

bool Shell::randomPointInCircle(dtCrowd* crowd, float* initialPoint, float maxRadius, float* targetPoint)
{
	const dtNavMeshQuery* navQuery = crowd->getNavMeshQuery();
	const dtQueryFilter* filter = crowd->getFilter(0);
	const float* ext = crowd->getQueryExtents();
	dtPolyRef targetRef;
	dtPolyRef nearestRef;
	float nearestPoint[3];

	dtStatus status = navQuery->findNearestPoly(initialPoint, ext, filter, &nearestRef, nearestPoint);
	if (dtStatusSucceed(status))
	{
		status = navQuery->findRandomPointAroundCircle(nearestRef, initialPoint, maxRadius, filter, random_float, &targetRef, targetPoint);
	}

	return dtStatusSucceed(status);
}

unsigned int Shell::addAreaFlags(dtTileCache* tileCache, dtCrowd* crowd, float* center, float* verts, int nverts, float height, unsigned short int flags)
{
	dtObstacleRef ref;
	tileCache->addFlag(center, verts, nverts, height, flags, crowd, &ref);

	return ref;
}

void Shell::removeAreaFlags(dtTileCache* tileCache, dtObstacleRef ref)
{
	tileCache->removeFlag(ref);
}


void Shell::setMonotonePartitioning(bool enabled)
{
	monotonePartitioning = enabled;
}

float random_float()
{
	return (double)rand() / (double)RAND_MAX;
}