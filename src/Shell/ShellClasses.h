#include "Recast.h"
#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "ChunkyTriMesh.h"
#include <fstream>
#include <string.h>
#include "Consts.h"
extern "C" {
#include "fastlz.h"
}



struct FastLZCompressor : public dtTileCacheCompressor
{
	virtual int maxCompressedSize(const int bufferSize)
	{
		return (int)(bufferSize* 1.05f);
	}

	virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
		unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
	{
		*compressedSize = fastlz_compress((const void *const)buffer, bufferSize, compressed);
		return DT_SUCCESS;
	}

	virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
		unsigned char* buffer, const int maxBufferSize, int* bufferSize)
	{
		*bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
		return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
	}
};

class Context : public rcContext
{
	std::ofstream outfile;

public:
	Context(char* logpath) :
		rcContext()
	{
		outfile.open(logpath, std::ios_base::app);
	}

	~Context()
	{
		outfile.close();
	}

	void doLog(const rcLogCategory category, const char* msg, const int /*len*/)
	{
		outfile << msg << "\n";
		outfile.flush();
		DebugCPP::Log(msg);
	}
};

struct TileCacheData
{
	unsigned char* data;
	int dataSize;
};

struct RasterizationContext
{
	RasterizationContext() :
		solid(0),
		triareas(0),
		lset(0),
		chf(0),
		ntiles(0)
	{
		memset(tiles, 0, sizeof(TileCacheData)*MAX_LAYERS);
	}

	~RasterizationContext()
	{
		rcFreeHeightField(solid);
		delete[] triareas;
		rcFreeHeightfieldLayerSet(lset);
		rcFreeCompactHeightfield(chf);
		for (int i = 0; i < MAX_LAYERS; ++i)
		{
			dtFree(tiles[i].data);
			tiles[i].data = 0;
		}
	}

	rcHeightfield* solid;
	unsigned char* triareas;
	rcHeightfieldLayerSet* lset;
	rcCompactHeightfield* chf;
	TileCacheData tiles[MAX_LAYERS];
	int ntiles;
};

struct LinearAllocator : public dtTileCacheAlloc
{
	unsigned char* buffer;
	int capacity;
	int top;
	int high;

	LinearAllocator(const int cap) : buffer(0), capacity(0), top(0), high(0)
	{
		resize(cap);
	}

	~LinearAllocator()
	{
		dtFree(buffer);
	}

	void resize(const int cap)
	{
		if (buffer) dtFree(buffer);
		buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
		capacity = cap;
	}

	virtual void reset()
	{
		high = dtMax(high, top);
		top = 0;
	}

	virtual void* alloc(const int size)
	{
		if (!buffer)
			return 0;
		if (top + size > capacity)
			return 0;
		unsigned char* mem = &buffer[top];
		top += size;
		return mem;
	}

	virtual void free(void* /*ptr*/)
	{
		// Empty
	}
};

struct ExtendedConfig
{
	float AgentHeight;
	float AgentRadius;
	float AgentMaxClimb;
	int MaxObstacles;
};

struct InputGeometry
{
	float* verts;
	int nverts;
	int* tris;
	int ntris;
};

struct TileCacheHolder
{
	rcConfig* cfg;
	ExtendedConfig* ecfg;
	InputGeometry* geom;
	rcChunkyTriMesh* chunkyMesh;
};

struct TileCacheSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams meshParams;
	dtTileCacheParams cacheParams;
};

struct TileCacheTileHeader
{
	dtCompressedTileRef tileRef;
	int dataSize;
};

struct ConvexVolume
{
	float verts[MAX_CONVEXVOL_PTS * 3];
	float hmin, hmax;
	int nverts;
	int area;
};

struct MeshProcess : public dtTileCacheMeshProcess
{
	InputGeometry* m_geom;
	unsigned short* flags;
	int* numFlags;

	inline MeshProcess(unsigned short* p_flags, int* p_numFlags) : m_geom(0)
	{
		flags = p_flags;
		numFlags = p_numFlags;
	}

	inline void init(InputGeometry* geom)
	{
		m_geom = geom;
	}

	virtual void process(struct dtNavMeshCreateParams* params,
		unsigned char* polyAreas, unsigned short* polyFlags)
	{
		for (int i = 0; i < params->polyCount; ++i)
		{
			polyFlags[i] = 0;

			if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
			{
				polyFlags[i] = flags[0];
			}
			else
			{
				for (int j = 1; j < *numFlags; ++j)
				{
					if (polyAreas[i] & flags[j])
					{
						polyFlags[i] |= flags[j];
					}
				}
			}
		}
	}

};
