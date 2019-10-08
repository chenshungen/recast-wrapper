#ifndef PATHFINDING_H
#define PATHFINDING_H 

#include <list>
#include "Navigation.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

#define TRYSEARCHTIMES  100

class PathFinding
{
private:
	Navigation* m_nav;
	dtNavMesh* m_navMesh;
	dtNavMeshQuery* m_navQuery;
	dtQueryFilter m_filter;
	dtStatus m_pathFindStatus;
	
	enum Mode
	{
		MODE_PATHFIND_FOLLOW,
		MODE_PATHFIND_STRAIGHT,
		MODE_DISTANCE_TO_WALL,
	};
	Mode m_mode;

	int m_straightPathOptions;

	static const int MAX_POLYS = 256;
	static const int MAX_SMOOTH = 2048;
	
	dtPolyRef m_startRef;
	dtPolyRef m_endRef;
	dtPolyRef m_polys[MAX_POLYS];
	dtPolyRef m_parent[MAX_POLYS];
	int m_npolys;
	float m_straightPath[MAX_POLYS*3];
	unsigned char m_straightPathFlags[MAX_POLYS];
	dtPolyRef m_straightPathPolys[MAX_POLYS];
	int m_nstraightPath;
	float m_polyPickExt[3];
	float m_smoothPath[MAX_SMOOTH*3];
	int m_nsmoothPath;
	float m_queryPoly[4*3];

	static const int MAX_RAND_POINTS = 1024;
	float m_randPoints[MAX_RAND_POINTS*3];
	int m_nrandPoints;
	bool m_randPointsInCircle;
	
	float m_spos[3];
	float m_epos[3];
	float m_hitPos[3];
	float m_hitNormal[3];
	bool m_hitResult;
	float m_distanceToWall;
	float m_neighbourhoodRadius;
	float m_randomRadius;
	bool m_sposSet;
	bool m_eposSet;

	int m_pathIterNum;
	dtPolyRef m_pathIterPolys[MAX_POLYS]; 
	int m_pathIterPolyCount;
	float m_prevIterPos[3], m_iterPos[3], m_steerPos[3], m_targetPos[3];

	static const int MAX_STEER_POINTS = 10;
	float m_steerPoints[MAX_STEER_POINTS*3];
	int m_steerPointCount;

	std::list<float> m_path;
	float step_size = 0.5f;
public:
	PathFinding();

	void recalc();

	void Init(Navigation* nav);

	void Reset();

	void SetEndPostion(const float* pos);

	void SetStartPostion(const float* pos);

	// pass start point postion and end point postion to parameter start, end
	float* FindSmoothPath(const float* start, const float* end, const float stepSize, int& pathSize);

	//  A circle that thinks pos is a center tp is tangency point between circle and wall
	float GetDistanceToClosestWall(const float* pos, float* tp = 0 /* tangency point */);

	// make random point in whole nav mesh num should less than MAX_RAND_POINTS
	float* MakeRandomPoints(int num);

	// make random point around pos and num should less than MAX_RAND_POINTS
	float* MakeRandomPointsAround(const float* pos, const float radius, int num);

	float* FindPath(const float* start, const float* end, int& point_num);


	void MakeRandomPoint(float* pos);

	bool PointInCircle(const float* center, const float radius, const float* pos);

	bool PointInRing(const float *center, const float *pos, const float inradius, const float outradius);

	bool MakeRandomPointInRing(const float *center, const float inradius, const float outradius, float *pos);
};

#endif
