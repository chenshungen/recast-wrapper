#ifndef NAVIGATION_H
#define NAVIGATION_H
#include <memory>
#include <list>

#include "Recast.h"
#include "Common.h"
 #include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourTileCache.h"
#include "DetourCrowd.h"
#include "ChunkyTriMesh.h"
#include "InputGeom.h"

class PathFinding;
class ConvexVolumeBuilder;
class CrowdManager;
class InputGeom;

class Navigation
{
private:
	InputGeomPtr m_geom;
	class dtNavMesh* m_navMesh;
	class dtNavMeshQuery* m_navQuery;
	class dtCrowd* m_crowd;

	float m_cellSize;
	float m_cellHeight;
	float m_agentHeight;
	float m_agentRadius;
	float m_agentMaxClimb;
	float m_agentMaxSlope;
	float m_regionMinSize;
	float m_regionMergeSize;
	float m_edgeMaxLen;
	float m_edgeMaxError;
	float m_vertsPerPoly;
	float m_detailSampleDist;
	float m_detailSampleMaxError;
	int m_partitionType;
	bool m_filterLowHangingObstacles;
	bool m_filterLedgeSpans;
	bool m_filterWalkableLowHeightSpans;
	BuildContext* m_ctx;
public:
	Navigation();
	virtual ~Navigation();

	void setContext(BuildContext* ctx) { m_ctx = ctx; }
	void DoUpdate(const float dt);
	void updateTileCache();

	InputGeomPtr getInputGeom() { return m_geom; }
	dtNavMesh* getNavMesh() { return m_navMesh; }
	dtNavMeshQuery* getNavMeshQuery() { return m_navQuery; }
	dtCrowd* getCrowd() { return m_crowd; }
	float getAgentRadius() { return m_agentRadius; }
	float getAgentHeight() { return m_agentHeight; }
	float getAgentClimb() { return m_agentMaxClimb; }
	void resetCommonSettings();
	void handleSettings();
	void ChangeMesh(InputGeomPtr geom);
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Navigation(const Navigation&);
	Navigation& operator=(const Navigation&);

private:
	struct LinearAllocator* m_talloc;
	struct FastLZCompressor* m_tcomp;
	struct MeshProcess* m_tmproc;
	class dtTileCache* m_tileCache;
	float m_cacheBuildTimeMs;
	int m_cacheCompressedSize;
	int m_cacheRawSize;
	int m_cacheLayerCount;
	int m_cacheBuildMemUsage;

	int m_maxTiles;
	int m_maxPolysPerTile;
	float m_tileSize;
public:
	int rasterizeTileLayers(const int tx, const int ty, const rcConfig& cfg, struct TileCacheData* tiles, const int maxTiles);

	void getTilePos(const float* pos, int& tx, int& ty);
	bool Load(const char* path);
	void saveAll(const char* path);
	bool loadAll(const char* path);

	bool addTempObstacle(const float* pos, const float radius);
	void removeTempObstacle(/*const float* sp,*/ const float* sq);
	void clearAllTempObstacles();

	void addOffMeshConnection(const float* s, const float* e, bool bidir);
	void deleteOffMeshConnection(const float* pos);
	void ValidateOffMeshConnection(const float* pos);
		
	void RebuildTileAt(const float* pts, int npts);

private:
	//ConvexVolumeBuilder* m_convexvolbuilder;
	PathFinding* m_pathfind;
	CrowdManager* m_crowdMgr;
public:
	//ConvexVolumeBuilder* GetConvexVolumeBuilder() { return m_convexvolbuilder;}
	PathFinding* PathFinder() { return m_pathfind; }
	CrowdManager* GetCrowdManager() { return m_crowdMgr; }
	bool IsValidate(const float* p) { return m_geom->IsValidate(p) ; }
	void getBounds(float* boundsMin, float* boundsMax);
	bool raycastHit(const float *p, float *raycastHit) { return m_geom->raycastHit(p, raycastHit); };
public:
	// pass start point postion and end point postion to parameter start, end
	float* FindSmoothPath(const float* start, const float* end, const float stepSize, int& pathSize);
	float* FindPath(const float* start, const float* end, int& point_num);

	////  A circle that thinks pos is a center tp is tangency point between circle and wall
	float GetDistanceToClosestWall(const float* pos, float* tp = 0);
	// make random point in whole nav mesh num should less than MAX_RAND_POINTS
	float* MakeRandomPoints(int num);

	// make random point around pos and num should less than MAX_RAND_POINTS
	float* MakeRandomPointsAround(const float* pos, const float radius, int num);

	void MakeRandomPoint(float* pos);

	bool MakeRandomPointInRing(const float *center, const float inradius, const float outradius, float *pos);
public:
	int		CreateAgent(const float* pos, const float radius = 0.0f);
	void	RemoveAgent(const int idx);
	void	ResetAgent(int idx, const float* pos);
	void	AgentMoveTo(float* target, int idx, bool vel = false);
	bool	GetAgentPosition(int idx, float* p);
	void	ResetMoveOp(int idx);
	bool	GetAgentVel(int idx, float*p);
	void	SetAgentMaxSpeed(int idx, float speed);
	float	GetAgentMaxSpeed(int idx);
	const	dtCrowdAgent* GetAgent(int idx);
};

Navigation* CreateNavSDK(std::string navmesh, std::string tilecache = "tilecache.bin");
#endif	// NAVIGATION_H
