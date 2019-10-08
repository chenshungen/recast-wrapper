#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include "CrowdManager.h"
#include "InputGeom.h"
#include "Navigation.h"
#include "DetourCrowd.h"
#include "DetourObstacleAvoidance.h"
#include "DetourCommon.h"
#include "DetourNode.h"
#include "Common.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

static bool isectSegAABB(const float* sp, const float* sq, const float* amin, const float* amax, float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;
	
	float d[3];
	dtVsub(d, sq, sp);
	tmin = 0;  // set to -FLT_MAX to get first hit on line
	tmax = FLT_MAX;		// set to max distance ray can travel (for segment)
	
	// For all three slabs
	for (int i = 0; i < 3; i++)
	{
		if (fabsf(d[i]) < EPS)
		{
			// Ray is parallel to slab. No hit if origin not within slab
			if (sp[i] < amin[i] || sp[i] > amax[i])
				return false;
		}
		else
		{
			// Compute intersection t value of ray with near and far plane of slab
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;
			// Make t1 be intersection with near plane, t2 with far plane
			if (t1 > t2) dtSwap(t1, t2);
			// Compute the intersection of slab intersections intervals
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			// Exit with no collision as soon as slab intersection becomes empty
			if (tmin > tmax) return false;
		}
	}
	
	return true;
}

static void getAgentBounds(const dtCrowdAgent* ag, float* bmin, float* bmax)
{
	const float* p = ag->npos;
	const float r = ag->params.radius;
	const float h = ag->params.height;
	bmin[0] = p[0] - r;
	bmin[1] = p[1];
	bmin[2] = p[2] - r;
	bmax[0] = p[0] + r;
	bmax[1] = p[1] + h;
	bmax[2] = p[2] + r;
}

CrowdManager::CrowdManager() : 
	m_nav(0),
	m_navmesh(0),
	m_crowd(0),
	m_targetRef(0),
	m_run(true)
{
	m_params.m_expandOptions = true;
	m_params.m_anticipateTurns = true;
	m_params.m_optimizeVis = true;
	m_params.m_optimizeTopo = true;
	m_params.m_obstacleAvoidance = false;
	m_params.m_obstacleAvoidanceType = 3.0f;
	m_params.m_separation = false;
	m_params.m_separationWeight = 2.0f;

	m_vod = dtAllocObstacleAvoidanceDebugData();
	m_vod->init(2048);
	
	memset(&m_agentDebug, 0, sizeof(m_agentDebug));
	m_agentDebug.idx = -1;
	m_agentDebug.vod = m_vod;
}

CrowdManager::~CrowdManager()
{
	dtFreeObstacleAvoidanceDebugData(m_vod);
}

void CrowdManager::init(Navigation* navigation)
{
	if (m_nav != navigation)
	{
		m_nav = navigation;
	}
	
	dtNavMesh* nav = m_nav->getNavMesh();
	dtCrowd* crowd = m_nav->getCrowd();
	
	if (nav && crowd && (m_navmesh != nav || m_crowd != crowd))
	{
		m_navmesh = nav;
		m_crowd = crowd;
	
		crowd->init(MAX_AGENTS, m_nav->getAgentRadius(), nav);
		
		// Make polygons with 'disabled' flag invalid.
		crowd->getEditableFilter(0)->setExcludeFlags(POLYFLAGS_DISABLED);
		
		// Setup local avoidance params to different qualities.
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
	}
}

void CrowdManager::DoUpdate(const float dt)
{
	if (m_run)
		updateTick(dt);
	//dtCrowd* crowd = m_nav->getCrowd();
	//for (int i = 0; i < crowd->getAgentCount(); ++i) 
	//{    
		//const dtCrowdAgent* ag = crowd->getAgent(i);
		//if (!ag->active) continue;

		//const float* pos = ag->npos;
		//printf("agent: %f  %f  %f \n", pos[0], pos[1], pos[2]);
	//}    
}

int CrowdManager::addAgent(const float* p, const float radius)
{
	if (!m_nav) return -1;
	dtCrowd* crowd = m_nav->getCrowd();
	
	dtCrowdAgentParams ap;
	memset(&ap, 0, sizeof(ap));
	ap.radius = m_nav->getAgentRadius();
	ap.radius = radius > 0.1f ? radius : ap.radius;
	ap.height = m_nav->getAgentHeight();
	ap.height = m_nav->getAgentHeight();
	ap.maxAcceleration = 100.0f;
	ap.maxSpeed = 2.0f;
	ap.collisionQueryRange = ap.radius * 12.0f;
	ap.pathOptimizationRange = ap.radius * 30.0f;
	ap.updateFlags = 0; 
	if (m_params.m_anticipateTurns)
		ap.updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
	if (m_params.m_optimizeVis)
		ap.updateFlags |= DT_CROWD_OPTIMIZE_VIS;
	if (m_params.m_optimizeTopo)
		ap.updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
	if (m_params.m_obstacleAvoidance)
		ap.updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
	if (m_params.m_separation)
		ap.updateFlags |= DT_CROWD_SEPARATION;
	ap.obstacleAvoidanceType = (unsigned char)m_params.m_obstacleAvoidanceType;
	ap.separationWeight = m_params.m_separationWeight;
	
	int idx = crowd->addAgent(p, &ap);
	if (idx != -1)
	{
		if (m_targetRef)
			crowd->requestMoveTarget(idx, m_targetRef, m_targetPos);
	}

	return idx;
}

void CrowdManager::removeAgent(const int idx)
{
	if (!m_nav) 
		return;
	dtCrowd* crowd = m_nav->getCrowd();
	if (!crowd)
		return;

	crowd->removeAgent(idx);
}

static void calcVel(float* vel, const float* pos, const float* tgt, const float speed)
{
	dtVsub(vel, tgt, pos);
	vel[1] = 0.0;
	dtVnormalize(vel);
	dtVscale(vel, vel, speed);
}

void CrowdManager::setMoveTarget(const float* p, bool adjust)
{
	if (!m_nav) return;
	
	// Find nearest point on navmesh and set move request to that location.
	dtNavMeshQuery* navquery = m_nav->getNavMeshQuery();
	dtCrowd* crowd = m_nav->getCrowd();
	const dtQueryFilter* filter = crowd->getFilter(0);
	const float* halfExtents = crowd->getQueryExtents();

	if (adjust)
	{
		float vel[3];
		for (int i = 0; i < crowd->getAgentCount(); ++i)
		{
			const dtCrowdAgent* ag = crowd->getAgent(i);
			if (!ag->active) continue;
			calcVel(vel, ag->npos, p, ag->params.maxSpeed);
			crowd->requestMoveVelocity(i, vel);
		}
	}
	else
	{
		navquery->findNearestPoly(p, halfExtents, filter, &m_targetRef, m_targetPos);
		for (int i = 0; i < crowd->getAgentCount(); ++i)
		{
			const dtCrowdAgent* ag = crowd->getAgent(i);
			if (!ag->active) continue;
			crowd->requestMoveTarget(i, m_targetRef, m_targetPos);
		}
		
	}
}

int CrowdManager::hitTestAgents(const float* s, const float* p)
{
	if (!m_nav) return -1;
	dtCrowd* crowd = m_nav->getCrowd();
	
	int isel = -1;
	float tsel = FLT_MAX;

	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active) continue;
		float bmin[3], bmax[3];
		getAgentBounds(ag, bmin, bmax);
		float tmin, tmax;
		if (isectSegAABB(s, p, bmin,bmax, tmin, tmax))
		{
			if (tmin > 0 && tmin < tsel)
			{
				isel = i;
				tsel = tmin;
			} 
		}
	}

	return isel;
}

void CrowdManager::updateAgentParams()
{
	if (!m_nav) return;
	dtCrowd* crowd = m_nav->getCrowd();
	if (!crowd) return;
	
	unsigned char updateFlags = 0;
	unsigned char obstacleAvoidanceType = 0;
	
	if (m_params.m_anticipateTurns)
		updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
	if (m_params.m_optimizeVis)
		updateFlags |= DT_CROWD_OPTIMIZE_VIS;
	if (m_params.m_optimizeTopo)
		updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
	if (m_params.m_obstacleAvoidance)
		updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
	if (m_params.m_obstacleAvoidance)
		updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
	if (m_params.m_separation)
		updateFlags |= DT_CROWD_SEPARATION;
	
	obstacleAvoidanceType = (unsigned char)m_params.m_obstacleAvoidanceType;
	
	dtCrowdAgentParams params;
	
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active) continue;
		memcpy(&params, &ag->params, sizeof(dtCrowdAgentParams));
		params.updateFlags = updateFlags;
		params.obstacleAvoidanceType = obstacleAvoidanceType;
		params.separationWeight = m_params.m_separationWeight;
		crowd->updateAgentParameters(i, &params);
	}	
}

void CrowdManager::updateTick(const float dt)
{
	if (!m_nav) return;
	dtNavMesh* nav = m_nav->getNavMesh();
	dtCrowd* crowd = m_nav->getCrowd();
	if (!nav || !crowd) return;
	
	crowd->update(dt, &m_agentDebug);
}

int CrowdManager::CreateAgent(const float* pos, const float radius)
{
	if (!m_nav) return -1;
	InputGeomPtr geom = m_nav->getInputGeom();
	if (!geom) return -1;
	if (! geom->IsValidate(pos))
	{
		return -1;
	}
	dtCrowd* crowd = m_nav->getCrowd();
	if (!crowd) 
		return -1;

	int idx = addAgent(pos, radius);
	ResetMoveOp(idx);
	return idx;

}

void CrowdManager::AgentMoveTo(float* target, int idx, bool vel)
{
	if (!m_nav) return;
	// Find nearest point on navmesh and set move request to that location.
	dtNavMeshQuery* navquery = m_nav->getNavMeshQuery();
	dtCrowd* crowd = m_nav->getCrowd();
	const dtQueryFilter* filter = crowd->getFilter(0);
	const float* halfExtents = crowd->getQueryExtents();

	if (vel)
	{
		const dtCrowdAgent* ag = crowd->getAgent(idx);
		if (ag && ag->active)
		{
			target[1] = 0.0f;
			dtVnormalize(target);
			dtVscale(target, target, ag->params.maxSpeed);
			crowd->requestMoveVelocity(idx, target);
		}
	}
	else 
	{
		navquery->findNearestPoly(target, halfExtents, filter, &m_targetRef, m_targetPos);
		const dtCrowdAgent* ag = crowd->getAgent(idx);
		if (ag && ag->active)
			crowd->requestMoveTarget(idx, m_targetRef, m_targetPos);
	}

}

bool CrowdManager::GetAgentPosition(int idx, float* p)
{
	dtCrowd* crowd = m_nav->getCrowd();
	const dtCrowdAgent* ag = crowd->getAgent(idx);
	if (ag && ag->active)
	{
		const float* pos = ag->npos;
		dtVcopy(p, pos);
		return true;
	}
	return false;
}

void CrowdManager::ResetMoveOp(int idx)
{
	if (!m_nav) return;
	dtCrowd* crowd = m_nav->getCrowd();
	if (!crowd) return;
	const dtCrowdAgent* ag = crowd->getAgent(idx);
	if (ag && ag->active)
		crowd->resetMoveTarget(idx);
}

bool CrowdManager::GetAgentVel(int idx, float*p)
{
	dtCrowd* crowd = m_nav->getCrowd();
	if (!crowd) 
		return false;
	const dtCrowdAgent* ag = crowd->getAgent(idx);
	if (ag && ag->active)
	{
		const float* vel = ag->vel;
		dtVcopy(p, vel);
		return true;
	}
	return false;
}

void CrowdManager::SetAgentMaxSpeed(int idx, float speed)
{
	dtCrowd* crowd = m_nav->getCrowd();
	if (!crowd) return;
	const dtCrowdAgent* ag = crowd->getAgent(idx);
	if (ag && ag->active)
	{
		dtCrowdAgentParams params;
		memcpy(&params, &ag->params, sizeof(dtCrowdAgentParams));
		params.maxSpeed  = speed;
		crowd->updateAgentParameters(idx, &params);
	}
}

float CrowdManager::GetAgentMaxSpeed(int idx)
{
	dtCrowd* crowd = m_nav->getCrowd();
	if (!crowd) return 0.0f;
	const dtCrowdAgent* ag = crowd->getAgent(idx);
	if (ag && ag->active)
		return ag->params.maxSpeed;
	return 0.0f;
}

const dtCrowdAgent* CrowdManager::GetAgent(int idx)
{
	dtCrowd* crowd = m_nav->getCrowd();
	if (!crowd)
		return NULL;
	const dtCrowdAgent* ag = crowd->getAgent(idx); 
	if (ag && ag->active)
		return ag;
	return NULL;
}

void CrowdManager::DebugAgentStatus(int idx)
{
	dtCrowd* crowd = m_nav->getCrowd(); 
	if(!crowd) return;
	const dtCrowdAgent* ag = crowd->getAgent(idx); 
	if (ag && ag->active)
	{
		//printf("targetRef: %d targetPos: %f  %f  %f targetPathqRef: %d  targetReplan: %d targetState: %d\n", ag->targetRef, ag->targetPos[0], ag->targetPos[1], ag->targetPos[2], ag->targetPathqRef, ag->targetReplan, ag->targetState);

		printf("pos: %f  %f  %f vel: %f  %f  %f \n", ag->npos[0], ag->npos[1], ag->npos[2], ag->vel[0], ag->vel[1], ag->vel[2]);
	}
}


void CrowdManager::ResetAgent(int idx, const float* pos)
{
	dtCrowd* crowd = m_nav->getCrowd();

	if (!crowd) 
		return;

	crowd->resetMoveTarget(idx);

	const dtCrowdAgent* ag = crowd->getAgent(idx);

	if (!ag)
		return;

	// TODO 这里可以传入改变的params
	crowd->resetAgent(idx, pos, &ag->params);
}

