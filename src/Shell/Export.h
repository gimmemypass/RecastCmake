#include "DetourCrowd.h"
#include "Shell.h"
#include "Debug.h"

#if defined(WIN32)
	#define DLL_EXPORT extern "C" __declspec(dllexport)
#elif defined(__GNUC__) && !defined(__clang__)
	#define DLL_EXPORT extern "C"
#else
	#define DLL_EXPORT extern "C"  __attribute__((visibility("default")))
#endif


// Debug
DLL_EXPORT bool RegisterDebugCallback(FuncCallBack cb);

DLL_EXPORT Shell* allocRecastShell(char* logpath);
DLL_EXPORT void freeShell(Shell* shell);

// Config
DLL_EXPORT rcConfig* DefaultConfig();
DLL_EXPORT int pointerSize();

// TileCache
DLL_EXPORT bool handleTileCacheBuild(Shell* shell, rcConfig* cfg, ExtendedConfig* ecfg, InputGeometry* geom, dtTileCache*& tileCache, dtNavMesh*& navMesh, dtNavMeshQuery*& navQuery);
DLL_EXPORT void addFlag(Shell* shell, unsigned short area, unsigned short cost);
DLL_EXPORT void getTileCacheHeaders(Shell* shell, TileCacheSetHeader& header, TileCacheTileHeader*& tilesHeader, dtTileCache* tileCache, dtNavMesh* navMesh);
DLL_EXPORT bool loadFromTileCacheHeaders(Shell* shell, TileCacheSetHeader* header, TileCacheTileHeader* tilesHeader, unsigned char* data, dtTileCache*& tileCache, dtNavMesh*& navMesh, dtNavMeshQuery*& navQuery);

// Class related
DLL_EXPORT dtCompressedTile* getTileCacheTile(Shell* shell, dtTileCache* tileCache, int i);
DLL_EXPORT dtObstacleRef addObstacle(Shell* shell, dtTileCache* tileCache, float* pos, float* verts, int nverts, float height, int* result);
DLL_EXPORT dtObstacleRef addCylinderObstacle(Shell* shell, dtTileCache* tileCache, float* pos, float radius, float height, int* result);
DLL_EXPORT void removeObstacle(Shell* shell, dtTileCache* tileCache, dtObstacleRef ref);
//DLL_EXPORT float* getObstacles(Shell* shell, dtTileCache* tc, int& nobstacles);
DLL_EXPORT unsigned char getObstacleState(Shell* shell, dtTileCache* tileCache, dtObstacleRef ref);

// Crowd
DLL_EXPORT dtCrowd* createCrowd(Shell* shell, int maxAgents, float maxRadius, dtNavMesh* navmesh);
DLL_EXPORT void setFilter(Shell* shell, dtCrowd* crowd, int filter, unsigned short include, unsigned short exclude);
DLL_EXPORT int addAgent(Shell* shell, dtCrowd* crowd, float* p, dtCrowdAgentParams* ap);
DLL_EXPORT dtCrowdAgent* getAgent(Shell* shell, dtCrowd* crowd, int idx);
DLL_EXPORT void updateAgent(Shell* shell, dtCrowd* crowd, int idx, dtCrowdAgentParams* ap);
DLL_EXPORT void removeAgent(Shell* shell, dtCrowd* crowd, int idx);
DLL_EXPORT void setMoveTarget(Shell* shell, dtNavMeshQuery* navquery, dtCrowd* crowd, int idx, float* p, bool adjust, int filterIndex);
DLL_EXPORT void resetPath(Shell* shell, dtCrowd* crowd, int idx);
DLL_EXPORT void updateTick(Shell* shell, dtTileCache* tileCache, dtNavMesh* nav, dtCrowd* crowd, float dt, float* positions, float* velocity, unsigned char* state, unsigned char* targetState, bool* partial, int& nagents);
DLL_EXPORT bool isPointValid(Shell* shell, dtCrowd* crowd, float* targetPoint);
DLL_EXPORT bool randomPoint(Shell* shell, dtCrowd* crowd, float* targetPoint);
DLL_EXPORT bool randomPointInCircle(Shell* shell, dtCrowd* crowd, float* initialPoint, float maxRadius, float* targetPoint);

//Config
//DLL_EXPORT void setMonotonePartitioning(Shell* shell, bool enabled);
//DLL_EXPORT void freeTileCache(dtNavMesh* navMesh, dtTileCache* tileCache);
// Build
//DLL_EXPORT rcPolyMesh* getPolyMesh(Shell* shell);
//DLL_EXPORT rcPolyMeshDetail* getPolyMeshDetail(Shell* shell);
//DLL_EXPORT bool handleBuild(Shell* shell, rcConfig* cfg, float* verts, int nverts, int* tris, int ntris);
//DLL_EXPORT bool createNavmesh(Shell* shell, rcConfig* cfg, rcPolyMesh* pmesh, rcPolyMeshDetail* dmesh, unsigned char*& navData, int& dataSize);
//TileCache
//DLL_EXPORT void addConvexVolume(Shell* shell, float* verts, int nverts, float hmax, float hmin, int area);
//Class related
//DLL_EXPORT dtMeshTile* getTile(Shell* shell, dtNavMesh* navmesh, int i);
//Crowd
//DLL_EXPORT unsigned int addAreaFlags(Shell* shell, dtTileCache* tileCache, dtCrowd* crowd, float* center, float* verts, int nverts, float height, unsigned short int flags);
//DLL_EXPORT void removeAreaFlags(Shell* shell, dtTileCache* tileCache, dtObstacleRef ref);

