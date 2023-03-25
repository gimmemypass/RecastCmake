// #include <vcruntime_string.h>
#include "ShellClasses.h"
#include "DetourCrowd.h"
#include "Debug.h"


class Shell
{
public:
    Shell(char* logpath);
    ~Shell();
    bool handleBuild(rcConfig* cfg, float* verts, int nverts, int* tris, int ntris);
    rcPolyMesh* getPolyMesh();
    rcPolyMeshDetail* getPolyMeshDetail();
    bool createNavmesh(rcConfig* cfg, rcPolyMesh* pmesh, rcPolyMeshDetail* dmesh, unsigned char*& navData, int& dataSize);
    int rasterizeTileLayers(Context* ctx, TileCacheHolder* holder,
        const int tx, const int ty,
        const rcConfig* cfg,
        TileCacheData* tiles,
        const int maxTiles);

    bool handleTileCacheBuild(rcConfig* cfg, ExtendedConfig* ecfg, InputGeometry* geom,
        dtTileCache*& tileCache, dtNavMesh*& navMesh, dtNavMeshQuery*& navQuery);
    void addFlag(unsigned short area, unsigned short cost);
    void addConvexVolume(float* verts, int nverts, float hmax, float hmin, int area);
    void getTileCacheHeaders(TileCacheSetHeader& header, TileCacheTileHeader*& tilesHeader, dtTileCache* tileCache, dtNavMesh* navMesh);
    bool loadFromTileCacheHeaders(TileCacheSetHeader* header, TileCacheTileHeader* tilesHeader, unsigned char* data, dtTileCache*& tileCache, dtNavMesh*& navMesh, dtNavMeshQuery*& navQuery);
    dtCompressedTile* getTileCacheTile(dtTileCache* tileCache, int i);
    dtMeshTile* getTile(dtNavMesh* navmesh, int i);
    dtObstacleRef addObstacle(dtTileCache* tileCache, float* pos, float* verts, int nverts, float height, int* result);
    dtObstacleRef addObstacle(dtTileCache* tileCache, float* pos, float radius, float height, int* result);
    unsigned char getObstacleState(dtTileCache* tileCache, dtObstacleRef ref);
    void removeObstacle(dtTileCache* tileCache, dtObstacleRef ref);
    //float* getObstacles(dtTileCache* tc, int& nobstacles);
    dtCrowd* createCrowd(int maxAgents, float maxRadius, dtNavMesh* navmesh);
    void setFilter(dtCrowd* crowd, int filter, unsigned short include, unsigned short exclude);
    int addAgent(dtCrowd* crowd, float* p, dtCrowdAgentParams* ap);
    dtCrowdAgent* getAgent(dtCrowd* crowd, int idx);
    void updateAgent(dtCrowd* crowd, int idx, dtCrowdAgentParams* ap);
    void removeAgent(dtCrowd* crowd, int idx);
    void calcVel(float* vel, const float* pos, const float* tgt, const float speed);
    void setMoveTarget(dtNavMeshQuery* navquery, dtCrowd* crowd, int idx, float* p, bool adjust, int filterIndex);
    void resetPath(dtCrowd* crowd, int idx);
    void updateTick(dtTileCache* tileCache, dtNavMesh* nav, dtCrowd* crowd, float dt, float* positions, float* velocity, unsigned char* state, unsigned char* targetState, bool* partial, int& nagents);
    bool isPointValid(dtCrowd* crowd, float* targetPoint);
    bool randomPoint(dtCrowd* crowd, float* targetPoint);
    bool randomPointInCircle(dtCrowd* crowd, float* initialPoint, float maxRadius, float* targetPoint);
    unsigned int addAreaFlags(dtTileCache* tileCache, dtCrowd* crowd, float* center, float* verts, int nverts, float height, unsigned short int flags);
    void removeAreaFlags(dtTileCache* tileCache, dtObstacleRef ref);
    void setMonotonePartitioning(bool enabled);

private:
    rcPolyMesh* pmesh;
    rcPolyMeshDetail* dmesh;
    Context* ctx;
    bool monotonePartitioning = false;
    int numConvexVolumes = 0;
    int numFlags = 0;
    ConvexVolume convexVolumes[MAX_CONVEX_VOLUMES];
    LinearAllocator* allocator = new LinearAllocator(5 * 1024 * 1024);
    FastLZCompressor* compressor = new FastLZCompressor();
    unsigned short* flags = new unsigned short[MAX_FLAGS];
    unsigned short* costs = new unsigned short[MAX_FLAGS];
    MeshProcess* processor = new MeshProcess(flags, &numFlags);
};
