#ifndef CROWDMANAGER_H
#define CROWDMANAGER_H

#include "Navigation.h"
#include "DetourNavMesh.h"
#include "DetourObstacleAvoidance.h"
#include "DetourCrowd.h"

struct CrowdParams
{
	bool m_expandOptions;
	bool m_anticipateTurns;
	bool m_optimizeVis;
	bool m_optimizeTopo;
	bool m_obstacleAvoidance;
	float m_obstacleAvoidanceType;
	bool m_separation;
	float m_separationWeight;
};

class CrowdManager
{
private:
	//Navigation* m_nav;
	Navigation* m_nav;
	dtNavMesh* m_navmesh;
	dtCrowd* m_crowd;

	float m_targetPos[3];
	dtPolyRef m_targetRef;

	dtCrowdAgentDebugInfo m_agentDebug;

	dtObstacleAvoidanceDebugData* m_vod;

	static const int MAX_AGENTS = 256;

	CrowdParams m_params;

	bool m_run;

public:
	CrowdManager();
	virtual ~CrowdManager();
	
	virtual void init(Navigation* nav);

	int addAgent(const float* pos, const float radius = 0.0f);
	void removeAgent(const int idx);
	void updateAgentParams();
	void setMoveTarget(const float* p, bool adjust);
	void updateTick(const float dt);
	void DoUpdate(const float dt);

	int hitTestAgents(const float* s, const float* p);
	inline CrowdParams* GetParams() { return &m_params; }

	int CreateAgent(const float* pos, const float radius = 0.0f);
	void AgentMoveTo(float* target, int idx, bool vel = false);
	bool GetAgentPosition(int idx, float* p);
	void ResetMoveOp(int idx);
	bool GetAgentVel(int idx, float*p);
	void SetAgentMaxSpeed(int idx, float speed);
	float GetAgentMaxSpeed(int idx);
	const dtCrowdAgent* GetAgent(int idx);
	void DebugAgentStatus(int idx);
	void ResetAgent(int idx, const float* pos);
};

#endif
