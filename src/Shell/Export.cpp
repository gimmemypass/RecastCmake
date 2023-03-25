#include "Export.h"

bool RegisterDebugCallback(FuncCallBack cb)
{
	DebugCPP::callbackInstance = cb;
	if (DebugCPP::callbackInstance == nullptr)
		throw;
	return DebugCPP::callbackInstance != nullptr;
}

Shell* allocRecastShell(char* logpath) 
{
	DebugCPP::Log("Trying to allocate shell");
	void* shell = dtAlloc(sizeof(Shell), DT_ALLOC_PERM);
	if (!shell) {
		DebugCPP::Log("Failed to create shell");
	}
	return new(shell)Shell(logpath);
}
void freeShell(Shell* shell)
{
	DebugCPP::Log("Free shell");
	dtFree(shell);
}

rcConfig* DefaultConfig()
{
	rcConfig* cfg = new rcConfig();

	// Default config
	float cellSize = 0.3f;
	float cellHeight = 0.2f;
	float agentHeight = 2.0f;
	float agentRadius = 0.6f;
	float agentMaxClimb = 0.8f;
	float agentMaxSlope = 20.0f;
	float regionMinSize = 8;
	float regionMergeSize = 20;
	float edgeMaxLen = 12.0f;
	float edgeMaxError = 1.3f;
	float vertsPerPoly = 6.0f;
	float detailSampleDist = 6.0f;
	float detailSampleMaxError = 1.0f;

	// Init build configuration from GUI
	memset(cfg, 0, sizeof(rcConfig));
	cfg->cs = cellSize;
	cfg->ch = cellHeight;
	cfg->tileSize = 48;
	cfg->walkableSlopeAngle = agentMaxSlope;
	cfg->walkableHeight = (int)ceilf(agentHeight / cfg->ch);
	cfg->walkableClimb = (int)floorf(agentMaxClimb / cfg->ch);
	cfg->walkableRadius = (int)ceilf(agentRadius / cfg->cs);
	cfg->maxEdgeLen = (int)(edgeMaxLen / cellSize);
	cfg->maxSimplificationError = edgeMaxError;
	cfg->minRegionArea = (int)rcSqr(regionMinSize);		// Note: area = size*size
	cfg->mergeRegionArea = (int)rcSqr(regionMergeSize);	// Note: area = size*size
	cfg->maxVertsPerPoly = (int)vertsPerPoly;
	cfg->detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
	cfg->detailSampleMaxError = cellHeight * detailSampleMaxError;
	return cfg;
}

int pointerSize()
{
	return sizeof(void*);
}

//void freeTileCache(dtNavMesh* navMesh, dtTileCache* tileCache)
//{
//	dtFreeNavMesh(navMesh);
//}

//void setMonotonePartitioning(Shell* shell, bool enabled)
//{
//	shell->setMonotonePartitioning(enabled);
//}

// Build
//rcPolyMesh* getPolyMesh(Shell* shell)
//{
//	return shell->getPolyMesh();
//}
//
//rcPolyMeshDetail* getPolyMeshDetail(Shell* shell)
//{
//	return shell->getPolyMeshDetail();
//}
//
//bool handleBuild(Shell* shell, rcConfig* cfg, float* verts, int nverts, int* tris, int ntris)
//{
//	return shell->handleBuild(cfg, verts, nverts, tris, ntris);
//}
//bool createNavmesh(Shell* shell, rcConfig* cfg, rcPolyMesh* pmesh, rcPolyMeshDetail* dmesh, unsigned char*& navData, int& dataSize)
//{
//	return shell->createNavmesh(cfg, pmesh, dmesh, navData, dataSize);
//}
// TileCache
bool handleTileCacheBuild(Shell* shell, rcConfig* cfg, ExtendedConfig* ecfg, InputGeometry* geom, dtTileCache*& tileCache, dtNavMesh*& navMesh, dtNavMeshQuery*& navQuery)
{
	return shell->handleTileCacheBuild(cfg, ecfg, geom, tileCache, navMesh, navQuery);
}
//void addConvexVolume(Shell* shell, float* verts, int nverts, float hmax, float hmin, int area)
//{
//	shell->addConvexVolume(verts, nverts, hmax, hmin, area);
//}
void addFlag(Shell* shell, unsigned short area, unsigned short cost)
{
	shell->addFlag(area, cost);
}
void getTileCacheHeaders(Shell* shell, TileCacheSetHeader& header, TileCacheTileHeader*& tilesHeader, dtTileCache* tileCache, dtNavMesh* navMesh)
{
	shell->getTileCacheHeaders(header, tilesHeader, tileCache, navMesh);
}
bool loadFromTileCacheHeaders(Shell* shell, TileCacheSetHeader* header, TileCacheTileHeader* tilesHeader, unsigned char* data, dtTileCache*& tileCache, dtNavMesh*& navMesh, dtNavMeshQuery*& navQuery)
{
	return shell->loadFromTileCacheHeaders(header, tilesHeader, data, tileCache, navMesh, navQuery);
}

// Class related
//dtMeshTile* getTile(Shell* shell, dtNavMesh* navmesh, int i)
//{
//	return shell->getTile(navmesh, i);
//}
dtCompressedTile* getTileCacheTile(Shell* shell, dtTileCache* tileCache, int i)
{
	return shell->getTileCacheTile(tileCache, i);
}
dtObstacleRef addObstacle(Shell* shell, dtTileCache* tileCache, float* pos, float* verts, int nverts, float height, int* result)
{
	return shell->addObstacle(tileCache, pos, verts, nverts, height, result);
}
dtObstacleRef addCylinderObstacle(Shell* shell, dtTileCache* tileCache, float* pos, float radius, float height, int* result)
{
	return shell->addObstacle(tileCache, pos, radius, height, result);	
}
unsigned char getObstacleState(Shell* shell, dtTileCache* tileCache, dtObstacleRef ref)
{
	return shell->getObstacleState(tileCache, ref);
}
void removeObstacle(Shell* shell, dtTileCache* tileCache, dtObstacleRef ref)
{
	shell->removeObstacle(tileCache, ref);
}
//float* getObstacles(Shell* shell, dtTileCache* tc, int& nobstacles)
//{
//	return shell->getObstacles(tc, nobstacles);
//}

// Crowd
dtCrowd* createCrowd(Shell* shell, int maxAgents, float maxRadius, dtNavMesh* navmesh)
{
	return shell->createCrowd(maxAgents, maxRadius, navmesh);
}
void setFilter(Shell* shell, dtCrowd* crowd, int filter, unsigned short include, unsigned short exclude)
{
	shell->setFilter(crowd, filter, include, exclude);
}
int addAgent(Shell* shell, dtCrowd* crowd, float* p, dtCrowdAgentParams* ap)
{
	return shell->addAgent(crowd, p, ap);
}
dtCrowdAgent* getAgent(Shell* shell, dtCrowd* crowd, int idx)
{
	return shell->getAgent(crowd, idx);
}
void updateAgent(Shell* shell, dtCrowd* crowd, int idx, dtCrowdAgentParams* ap)
{
	shell->updateAgent(crowd, idx, ap);
}
void removeAgent(Shell* shell, dtCrowd* crowd, int idx)
{
	shell->removeAgent(crowd, idx);
}
void setMoveTarget(Shell* shell, dtNavMeshQuery* navquery, dtCrowd* crowd, int idx, float* p, bool adjust, int filterIndex)
{
	shell->setMoveTarget(navquery, crowd, idx, p, adjust, filterIndex);
}
void resetPath(Shell* shell, dtCrowd* crowd, int idx)
{
	shell->resetPath(crowd, idx);
}
void updateTick(Shell* shell, dtTileCache* tileCache, dtNavMesh* nav, dtCrowd* crowd, float dt, float* positions, float* velocity, unsigned char* state, unsigned char* targetState, bool* partial, int& nagents)
{
	shell->updateTick(tileCache, nav, crowd, dt, positions, velocity, state, targetState, partial, nagents);
}
bool isPointValid(Shell* shell, dtCrowd* crowd, float* targetPoint)
{
	return shell->isPointValid(crowd, targetPoint);
}
bool randomPoint(Shell* shell, dtCrowd* crowd, float* targetPoint)
{
	return shell->randomPoint(crowd, targetPoint);
}
bool randomPointInCircle(Shell* shell, dtCrowd* crowd, float* initialPoint, float maxRadius, float* targetPoint)
{
	return shell->randomPointInCircle(crowd, initialPoint, maxRadius, targetPoint);
}
//unsigned int addAreaFlags(Shell* shell, dtTileCache* tileCache, dtCrowd* crowd, float* center, float* verts, int nverts, float height, unsigned short int flags)
//{
//	return shell->addAreaFlags(tileCache, crowd, center, verts, nverts, height, flags);
//}
//void removeAreaFlags(Shell* shell, dtTileCache* tileCache, dtObstacleRef ref)
//{
//	shell->removeAreaFlags(tileCache, ref);
//}

