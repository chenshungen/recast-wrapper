#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "PathFinding.h"
#include "Navigation.h"
#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourCommon.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

// Returns a random number [0..1]
static float frand()
{
//	return ((float)(rand() & 0xffff)/(float)0xffff);
	return (float)rand()/(float)RAND_MAX;
}


inline bool inRange(const float* v1, const float* v2, const float r, const float h)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1];
	const float dz = v2[2] - v1[2];
	return (dx*dx + dz*dz) < r*r && fabsf(dy) < h;
}


static int fixupCorridor(dtPolyRef* path, const int npath, const int maxPath, const dtPolyRef* visited, const int nvisited)
{
	int furthestPath = -1;
	int furthestVisited = -1;

	// Find furthest common polygon.
	for (int i = npath-1; i >= 0; --i)
	{
		bool found = false;
		for (int j = nvisited-1; j >= 0; --j)
		{
			if (path[i] == visited[j])
			{
				furthestPath = i;
				furthestVisited = j;
				found = true;
			}
		}
		if (found)
			break;
	}

	// If no intersection found just return current path. 
	if (furthestPath == -1 || furthestVisited == -1)
		return npath;

	// Concatenate paths.	

	// Adjust beginning of the buffer to include the visited.
	const int req = nvisited - furthestVisited;
	const int orig = rcMin(furthestPath+1, npath);
	int size = rcMax(0, npath-orig);
	if (req+size > maxPath)
		size = maxPath-req;
	if (size)
		memmove(path+req, path+orig, size*sizeof(dtPolyRef));

	// Store visited
	for (int i = 0; i < req; ++i)
		path[i] = visited[(nvisited-1)-i];				

	return req+size;
}

//// This function checks if the path has a small U-turn, that is,
//// a polygon further in the path is adjacent to the first polygon
//// in the path. If that happens, a shortcut is taken.
//// This can happen if the target (T) location is at tile boundary,
//// and we're (S) approaching it parallel to the tile edge.
//// The choice at the vertex can be arbitrary, 
////  +---+---+
////  |:::|:::|
////  +-S-+-T-+
////  |:::|   | <-- the step can end up in here, resulting U-turn path.
////  +---+---+
static int fixupShortcuts(dtPolyRef* path, int npath, dtNavMeshQuery* navQuery)
{
	if (npath < 3)
		return npath;

	// Get connected polygons
	static const int maxNeis = 16;
	dtPolyRef neis[maxNeis];
	int nneis = 0;

	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (dtStatusFailed(navQuery->getAttachedNavMesh()->getTileAndPolyByRef(path[0], &tile, &poly)))
		return npath;
	
	for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
	{
		const dtLink* link = &tile->links[k];
		if (link->ref != 0)
		{
			if (nneis < maxNeis)
				neis[nneis++] = link->ref;
		}
	}

	// If any of the neighbour polygons is within the next few polygons
	// in the path, short cut to that polygon directly.
	static const int maxLookAhead = 6;
	int cut = 0;
	for (int i = dtMin(maxLookAhead, npath) - 1; i > 1 && cut == 0; i--) 
	{
		for (int j = 0; j < nneis; j++)
		{
			if (path[i] == neis[j]) {
				cut = i;
				break;
			}
		}
	}
	if (cut > 1)
	{
		int offset = cut-1;
		npath -= offset;
		for (int i = 1; i < npath; i++)
			path[i] = path[i+offset];
	}

	return npath;
}

static bool getSteerTarget(dtNavMeshQuery* navQuery, const float* startPos, const float* endPos,
						   const float minTargetDist,
						   const dtPolyRef* path, const int pathSize,
						   float* steerPos, unsigned char& steerPosFlag, dtPolyRef& steerPosRef,
						   float* outPoints = 0, int* outPointCount = 0)							 
{
	// Find steer target.
	static const int MAX_STEER_POINTS = 3;
	float steerPath[MAX_STEER_POINTS*3];
	unsigned char steerPathFlags[MAX_STEER_POINTS];
	dtPolyRef steerPathPolys[MAX_STEER_POINTS];
	int nsteerPath = 0;
	navQuery->findStraightPath(startPos, endPos, path, pathSize,
							   steerPath, steerPathFlags, steerPathPolys, &nsteerPath, MAX_STEER_POINTS);
	if (!nsteerPath)
		return false;
		
	if (outPoints && outPointCount)
	{
		*outPointCount = nsteerPath;
		for (int i = 0; i < nsteerPath; ++i)
			dtVcopy(&outPoints[i*3], &steerPath[i*3]);
	}

	
	// Find vertex far enough to steer to.
	int ns = 0;
	while (ns < nsteerPath)
	{
		// Stop at Off-Mesh link or when point is further than slop away.
		if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
			!inRange(&steerPath[ns*3], startPos, minTargetDist, 1000.0f))
			break;
		ns++;
	}
	// Failed to find good point to steer to.
	if (ns >= nsteerPath)
		return false;
	
	dtVcopy(steerPos, &steerPath[ns*3]);
	steerPos[1] = startPos[1];
	steerPosFlag = steerPathFlags[ns];
	steerPosRef = steerPathPolys[ns];
	
	return true;
}


PathFinding::PathFinding() :
	m_nav(0),
	m_navMesh(0),
	m_navQuery(0),
	m_pathFindStatus(DT_FAILURE),
	m_mode(MODE_PATHFIND_FOLLOW),
	m_straightPathOptions(0),
	m_startRef(0),
	m_endRef(0),
	m_npolys(0),
	m_nstraightPath(0),
	m_nsmoothPath(0),
	m_nrandPoints(0),
	m_randPointsInCircle(false),
	m_hitResult(false),
	m_distanceToWall(0),
	m_sposSet(false),
	m_eposSet(false),
	m_pathIterNum(0),
	m_pathIterPolyCount(0),
	m_steerPointCount(0)
{
	m_filter.setIncludeFlags(POLYFLAGS_ALL ^ POLYFLAGS_DISABLED);
	m_filter.setExcludeFlags(0);

	m_polyPickExt[0] = 2;
	m_polyPickExt[1] = 4;
	m_polyPickExt[2] = 2;
	
	m_neighbourhoodRadius = 2.5f;
	m_randomRadius = 5.0f;
}

void PathFinding::Init(Navigation* nav) 
{
	m_nav = nav;
	m_navMesh = m_nav->getNavMesh();
	m_navQuery = m_nav->getNavMeshQuery();
	recalc();

	if (m_navQuery)
	{
		// Change costs.
		m_filter.setAreaCost(POLYAREA_GROUND, 1.0f);
		m_filter.setAreaCost(POLYAREA_WATER, 10.0f);
		m_filter.setAreaCost(POLYAREA_ROAD, 1.0f);
		m_filter.setAreaCost(POLYAREA_DOOR, 1.0f);
		m_filter.setAreaCost(POLYAREA_GRASS, 2.0f);
		m_filter.setAreaCost(POLYAREA_JUMP, 1.5f);
	}
	
	m_neighbourhoodRadius = m_nav->getAgentRadius() * 20.0f;
	m_randomRadius = m_nav->getAgentRadius() * 30.0f;
}

void PathFinding::SetStartPostion(const float* pos) 
{
	dtVcopy(m_spos, pos);
	m_sposSet = true;
}

void PathFinding::SetEndPostion(const float* pos)
{
	dtVcopy(m_epos, pos);
	m_eposSet = true;
}

void PathFinding::Reset()
{
	m_startRef = 0;
	m_endRef = 0;
	m_npolys = 0;
	m_nsmoothPath = 0;
	memset(m_hitPos, 0, sizeof(m_hitPos));
	memset(m_hitNormal, 0, sizeof(m_hitNormal));
	m_distanceToWall = 0;
}

float PathFinding::GetDistanceToClosestWall(const float* pos, float* tp) 
{
	m_mode = MODE_DISTANCE_TO_WALL;
	SetStartPostion(pos); 
	recalc();
	dtVcopy(tp, m_hitPos);
	return m_distanceToWall;
}

float* PathFinding::FindSmoothPath(const float* start, const float* end, const float stepSize, int& pathSize)
{
	m_mode = MODE_PATHFIND_FOLLOW;
	SetStartPostion(start);
	SetEndPostion(end);
	step_size = stepSize;
	recalc();
	pathSize = m_nsmoothPath;
	return m_smoothPath;
}

float* PathFinding::FindPath(const float* start, const float* end, int& point_num)
{
	m_mode = MODE_PATHFIND_STRAIGHT;
	SetStartPostion(start);
	SetEndPostion(end);
	recalc();
	point_num = m_nstraightPath;
	return m_straightPath;
}

void PathFinding::MakeRandomPoint(float* pos)
{
	float pt[3];
	dtPolyRef ref; 
	dtStatus status = m_navQuery->findRandomPoint(&m_filter, frand, &ref, pt); 
	if (dtStatusSucceed(status))
	{    
		dtVcopy(pos, pt); 
	}    
}

float* PathFinding::MakeRandomPoints(int num)
{
	m_randPointsInCircle = false;
	m_nrandPoints = 0; 
	for (int i = 0; i < num && i < MAX_RAND_POINTS; i++) 
	{    
		float pt[3];
		dtPolyRef ref; 
		dtStatus status = m_navQuery->findRandomPoint(&m_filter, frand, &ref, pt); 
		if (dtStatusSucceed(status))
		{    
			dtVcopy(&m_randPoints[m_nrandPoints*3], pt); 
			m_nrandPoints++;
		}    
	}    
	return m_randPoints;
}

bool PathFinding::PointInCircle(const float* center, const float radius, const float* pos)
{
	bool inCircle = false;
	if (dtVdist(center, pos) <= radius)
		inCircle = true;
	//printf("find inCircle ? %d center: %f  %f  %f  point: %f  %f  %f \n", inCircle, center[0], center[1], center[2], pos[0], pos[1], pos[2]);
	return inCircle;
}

bool PathFinding::PointInRing(const float *center, const float *pos, const float inradius, const float outradius)
{
	bool inRing = false;
	if (dtVdist(center, pos) <= outradius && dtVdist(center, pos) >= inradius)
		inRing = true;
	//printf("find a position in ring %d \n", inRing);
	return inRing;
}

static double rangeRandom(int min,int max)
{
    double m1=(double)(rand()%101)/101;                     // 计算 0，1之间的随机小数,得到的值域近似为(0,1)
    min++;                                                  // 将区间变为(min+1,max),
    double m2=(double)((rand()%(max-min+1))+min);           // 计算 min+1,max 之间的随机整数，得到的值域为[min+1,max]
    m2=m2-1;                                                // 令值域为[min,max-1]
    return m1+m2;                                           // 返回值域为(min,max),为所求随机浮点数
}

bool PathFinding::MakeRandomPointInRing(const float *center, const float inradius, const float outradius, float *pos)
{
    int try_count = 100;
    float target[3] = {0, 0, 0}; 
    while(try_count--)
    {   
        float x = rangeRandom(-1, 1); 
        float z = rangeRandom(-1, 1); 
        float dist = rangeRandom(inradius, outradius);
        float direction[3] = {x, 0, z}; 
        dtVnormalize(direction);
        dtVscale(direction, direction, dist);
        dtVadd(target, center, direction);

		float raycastHit[3];
        if (m_nav->raycastHit(target, raycastHit))
		{
			dtVcopy(target, raycastHit);
            break;
		}
    }   
    dtVcopy(pos, target);
	return true;
}

// 建议radius大于1.0f
float* PathFinding::MakeRandomPointsAround(const float* pos, const float radius, int num)
{
	SetStartPostion(pos);
	if (m_sposSet)
		m_navQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, &m_startRef, 0);
	else
		m_startRef = 0;
	
	m_nrandPoints = 0; 
	m_randPointsInCircle = true;
	for (int i = 0; i < num && i < MAX_RAND_POINTS; i++) 
	{    
		float pt[3];
		dtPolyRef ref; 
		dtStatus status;
		for (int k = 0; k < TRYSEARCHTIMES; k++)
		{
			status = m_navQuery->findRandomPointAroundCircle(m_startRef, m_spos, radius, &m_filter, frand, &ref, pt); 
			if (PointInCircle(pos, radius, pt))
			{
				//printf("point in circle...");
				break;
			}
		}
		if (dtStatusSucceed(status))
		{    
			dtVcopy(&m_randPoints[m_nrandPoints*3], pt); 
			m_nrandPoints++;
		}    
	}    

	return m_randPoints;
}

void PathFinding::recalc()
{
	if (!m_navMesh)
		return;
	
	if (m_sposSet)
		m_navQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, &m_startRef, 0);
	else
		m_startRef = 0;
	
	if (m_eposSet)
		m_navQuery->findNearestPoly(m_epos, m_polyPickExt, &m_filter, &m_endRef, 0);
	else
		m_endRef = 0;
	
	m_pathFindStatus = DT_FAILURE;
	
	if (m_mode == MODE_PATHFIND_FOLLOW)
	{
		m_pathIterNum = 0;
		if (m_sposSet && m_eposSet && m_startRef && m_endRef)
		{
#ifdef DUMP_REQS
			printf("pi  %f %f %f  %f %f %f  0x%x 0x%x\n", m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2], m_filter.getIncludeFlags(), m_filter.getExcludeFlags()); 
#endif
			m_navQuery->findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, m_polys, &m_npolys, MAX_POLYS);

			m_nsmoothPath = 0;

			if (m_npolys)
			{
				// Iterate over the path to find smooth path on the detail mesh surface.
				dtPolyRef polys[MAX_POLYS];
				memcpy(polys, m_polys, sizeof(dtPolyRef)*m_npolys); 
				int npolys = m_npolys;
				
				float iterPos[3], targetPos[3];
				m_navQuery->closestPointOnPoly(m_startRef, m_spos, iterPos, 0);
				m_navQuery->closestPointOnPoly(polys[npolys-1], m_epos, targetPos, 0);
				
				//static const float STEP_SIZE = 0.5f;
				static const float SLOP = 0.01f;
				
				m_nsmoothPath = 0;
				
				dtVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
				m_nsmoothPath++;
				
				// Move towards target a small advancement at a time until target reached or
				// when ran out of memory to store the path.
				while (npolys && m_nsmoothPath < MAX_SMOOTH)
				{
					// Find location to steer towards.
					float steerPos[3];
					unsigned char steerPosFlag;
					dtPolyRef steerPosRef;
					
					if (!getSteerTarget(m_navQuery, iterPos, targetPos, SLOP, polys, npolys, steerPos, steerPosFlag, steerPosRef))
						break;
					
					bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
					bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
					
					// Find movement delta.
					float delta[3], len;
					dtVsub(delta, steerPos, iterPos);
					len = dtMathSqrtf(dtVdot(delta, delta));
					// If the steer target is end of path or off-mesh link, do not move past the location.
					//if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
					if ((endOfPath || offMeshConnection) && len < step_size)
						len = 1;
					else
						//len = STEP_SIZE / len;
						len = step_size / len;
					float moveTgt[3];
					dtVmad(moveTgt, iterPos, delta, len);
					
					// Move
					float result[3];
					dtPolyRef visited[16];
					int nvisited = 0;
					m_navQuery->moveAlongSurface(polys[0], iterPos, moveTgt, &m_filter,
												 result, visited, &nvisited, 16);

					npolys = fixupCorridor(polys, npolys, MAX_POLYS, visited, nvisited);
					npolys = fixupShortcuts(polys, npolys, m_navQuery);

					float h = 0;
					m_navQuery->getPolyHeight(polys[0], result, &h);
					result[1] = h;
					dtVcopy(iterPos, result);

					// Handle end of path and off-mesh links when close enough.
					if (endOfPath && inRange(iterPos, steerPos, SLOP, 1.0f))
					{
						// Reached end of path.
						dtVcopy(iterPos, targetPos);
						if (m_nsmoothPath < MAX_SMOOTH)
						{
							dtVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
							m_nsmoothPath++;
						}
						break;
					}
					else if (offMeshConnection && inRange(iterPos, steerPos, SLOP, 1.0f))
					{
						// Reached off-mesh connection.
						float startPos[3], endPos[3];
						
						// Advance the path up to and over the off-mesh connection.
						dtPolyRef prevRef = 0, polyRef = polys[0];
						int npos = 0;
						while (npos < npolys && polyRef != steerPosRef)
						{
							prevRef = polyRef;
							polyRef = polys[npos];
							npos++;
						}
						for (int i = npos; i < npolys; ++i)
							polys[i-npos] = polys[i];
						npolys -= npos;
						
						// Handle the connection.
						dtStatus status = m_navMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos);
						if (dtStatusSucceed(status))
						{
							if (m_nsmoothPath < MAX_SMOOTH)
							{
								dtVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
								m_nsmoothPath++;
								// Hack to make the dotted path not visible during off-mesh connection.
								if (m_nsmoothPath & 1)
								{
									dtVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
									m_nsmoothPath++;
								}
							}
							// Move position at the other side of the off-mesh link.
							dtVcopy(iterPos, endPos);
							float eh = 0.0f;
							m_navQuery->getPolyHeight(polys[0], iterPos, &eh);
							iterPos[1] = eh;
						}
					}
					
					// Store results.
					if (m_nsmoothPath < MAX_SMOOTH)
					{
						dtVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
						m_nsmoothPath++;
					}
				}
			}

		}
		else
		{
			m_npolys = 0;
			m_nsmoothPath = 0;
		}
	}
	else if (m_mode == MODE_PATHFIND_STRAIGHT)
	{
		if (m_sposSet && m_eposSet && m_startRef && m_endRef)
		{
#ifdef DUMP_REQS
			printf("ps  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags()); 
#endif
			m_navQuery->findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, m_polys, &m_npolys, MAX_POLYS);
			m_nstraightPath = 0;
			if (m_npolys)
			{
				// In case of partial path, make sure the end point is clamped to the last polygon.
				float epos[3];
				dtVcopy(epos, m_epos);
				if (m_polys[m_npolys-1] != m_endRef)
					m_navQuery->closestPointOnPoly(m_polys[m_npolys-1], m_epos, epos, 0);
				
				m_navQuery->findStraightPath(m_spos, epos, m_polys, m_npolys,
											 m_straightPath, m_straightPathFlags,
											 m_straightPathPolys, &m_nstraightPath, MAX_POLYS, m_straightPathOptions);
			}
		}
		else
		{
			m_npolys = 0;
			m_nstraightPath = 0;
		}
	}
	else if (m_mode == MODE_DISTANCE_TO_WALL)
	{
		m_distanceToWall = 0;
		if (m_sposSet && m_startRef)
		{
#ifdef DUMP_REQS
			printf("dw  %f %f %f  %f  0x%x 0x%x\n", m_spos[0],m_spos[1],m_spos[2], 100.0f, m_filter.getIncludeFlags(), m_filter.getExcludeFlags()); 
#endif
			m_distanceToWall = 0.0f;
			m_navQuery->findDistanceToWall(m_startRef, m_spos, 100.0f, &m_filter, &m_distanceToWall, m_hitPos, m_hitNormal);
		}
	}
}
