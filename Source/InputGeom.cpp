//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <algorithm>
#include "Recast.h"
#include "InputGeom.h"
#include "ChunkyTriMesh.h"
#include "MeshLoaderObj.h"
#include "DetourNavMesh.h"
#include "Navigation.h"

static bool intersectSegmentTriangle(const float* sp, const float* sq,
									 const float* a, const float* b, const float* c,
									 float &t)
{
	float v, w;
	float ab[3], ac[3], qp[3], ap[3], norm[3], e[3];
	rcVsub(ab, b, a);
	rcVsub(ac, c, a);
	rcVsub(qp, sp, sq);
	
	// Compute triangle normal. Can be precalculated or cached if
	// intersecting multiple segments against the same triangle
	rcVcross(norm, ab, ac);
	
	// Compute denominator d. If d <= 0, segment is parallel to or points
	// away from triangle, so exit early
	float d = rcVdot(qp, norm);
	if (d <= 0.0f) return false;
	
	// Compute intersection t value of pq with plane of triangle. A ray
	// intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
	// dividing by d until intersection has been found to pierce triangle
	rcVsub(ap, sp, a);
	t = rcVdot(ap, norm);
	if (t < 0.0f) return false;
	if (t > d) return false; // For segment; exclude this code line for a ray test
	
	// Compute barycentric coordinate components and test if within bounds
	rcVcross(e, qp, ap);
	v = rcVdot(ac, e);
	if (v < 0.0f || v > d) return false;
	w = -rcVdot(ab, e);
	if (w < 0.0f || v + w > d) return false;
	
	// Segment/ray intersects triangle. Perform delayed division
	t /= d;
	
	return true;
}

std::map<std::string, std::shared_ptr<InputGeom>> InputGeom::m_inputGeoms;
InputGeom::InputGeom() :
	m_chunkyMesh(0),
	m_mesh(0),
	m_offMeshConCount(0),
	m_volumeCount(0)
{
}

InputGeom::~InputGeom()
{
	delete m_chunkyMesh;
	delete m_mesh;
}
		
bool InputGeom::loadMesh(const std::string& filepath)
{
	if (m_mesh)
	{
		delete m_chunkyMesh;
		m_chunkyMesh = 0;
		delete m_mesh;
		m_mesh = 0;
	}
	m_offMeshConCount = 0;
	m_volumeCount = 0;
	
	m_mesh = new rcMeshLoaderObj;
	if (!m_mesh)
	{
		printf("loadMesh: Out of memory 'm_mesh'.\n");
		return false;
	}
	if (!m_mesh->load(filepath))
	{
		printf("RC_LOG_ERROR buildTiledNavigation: Could not load '%s' \n", filepath.c_str());
		return false;
	}

	rcCalcBounds(m_mesh->getVerts(), m_mesh->getVertCount(), m_meshBMin, m_meshBMax);

	m_chunkyMesh = new rcChunkyTriMesh;
	if (!m_chunkyMesh)
	{
		printf("RC_LOG_ERROR buildTiledNavigation: Out of memory 'm_chunkyMesh'.\n");
		return false;
	}
	if (!rcCreateChunkyTriMesh(m_mesh->getVerts(), m_mesh->getTris(), m_mesh->getTriCount(), 256, m_chunkyMesh))
	{
		printf("RC_LOG_ERROR buildTiledNavigation: Failed to build chunky mesh.\n");
		return false;
	}		

	return true;
}

bool InputGeom::load(const std::string& filepath)
{
	size_t extensionPos = filepath.find_last_of('.');
	if (extensionPos == std::string::npos)
		return false;

	std::string extension = filepath.substr(extensionPos);
	std::transform(extension.begin(), extension.end(), extension.begin(), tolower);

	if (extension == ".obj")
	{
		bool ret = loadMesh(filepath);
		if (ret)
			m_inputGeoms.insert(std::make_pair(filepath, shared_from_this()));
		return ret;
	}

	return false;
}

static bool isectSegAABB(const float* sp, const float* sq,
						 const float* amin, const float* amax,
						 float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;
	
	float d[3];
	d[0] = sq[0] - sp[0];
	d[1] = sq[1] - sp[1];
	d[2] = sq[2] - sp[2];
	tmin = 0.0;
	tmax = 1.0f;
	
	for (int i = 0; i < 3; i++)
	{
		if (fabsf(d[i]) < EPS)
		{
			if (sp[i] < amin[i] || sp[i] > amax[i])
				return false;
		}
		else
		{
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;
			if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			if (tmin > tmax) return false;
		}
	}
	
	return true;
}


bool InputGeom::raycastMesh(float* src, float* dst, float& tmin)
{
	float dir[3];
	rcVsub(dir, dst, src);

	// Prune hit ray.
	float btmin, btmax;
	if (!isectSegAABB(src, dst, m_meshBMin, m_meshBMax, btmin, btmax))
		return false;
	float p[2], q[2];
	p[0] = src[0] + (dst[0]-src[0])*btmin;
	p[1] = src[2] + (dst[2]-src[2])*btmin;
	q[0] = src[0] + (dst[0]-src[0])*btmax;
	q[1] = src[2] + (dst[2]-src[2])*btmax;
	
	int cid[512];
	const int ncid = rcGetChunksOverlappingSegment(m_chunkyMesh, p, q, cid, 512);
	if (!ncid)
		return false;
	
	tmin = 1.0f;
	bool hit = false;
	const float* verts = m_mesh->getVerts();
	
	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = m_chunkyMesh->nodes[cid[i]];
		const int* tris = &m_chunkyMesh->tris[node.i*3];
		const int ntris = node.n;

		for (int j = 0; j < ntris*3; j += 3)
		{
			float t = 1;
			if (intersectSegmentTriangle(src, dst, &verts[tris[j]*3], &verts[tris[j+1]*3], &verts[tris[j+2]*3], t))
			{
				if (t < tmin)
					tmin = t;
				hit = true;
			}
		}
	}
	
	return hit;
}

void InputGeom::addOffMeshConnection(const float* spos, const float* epos, const float rad,
									 unsigned char bidir, unsigned char area, unsigned short flags)
{
	if (m_offMeshConCount >= MAX_OFFMESH_CONNECTIONS) return;
	float* v = &m_offMeshConVerts[m_offMeshConCount*3*2];
	m_offMeshConRads[m_offMeshConCount] = rad;
	m_offMeshConDirs[m_offMeshConCount] = bidir;
	m_offMeshConAreas[m_offMeshConCount] = area;
	m_offMeshConFlags[m_offMeshConCount] = flags;
	m_offMeshConId[m_offMeshConCount] = 1000 + m_offMeshConCount;
	rcVcopy(&v[0], spos);
	rcVcopy(&v[3], epos);
	m_offMeshConCount++;
}

void InputGeom::deleteOffMeshConnection(int i)
{
	m_offMeshConCount--;
	float* src = &m_offMeshConVerts[m_offMeshConCount*3*2];
	float* dst = &m_offMeshConVerts[i*3*2];
	rcVcopy(&dst[0], &src[0]);
	rcVcopy(&dst[3], &src[3]);
	m_offMeshConRads[i] = m_offMeshConRads[m_offMeshConCount];
	m_offMeshConDirs[i] = m_offMeshConDirs[m_offMeshConCount];
	m_offMeshConAreas[i] = m_offMeshConAreas[m_offMeshConCount];
	m_offMeshConFlags[i] = m_offMeshConFlags[m_offMeshConCount];
}


void InputGeom::addConvexVolume(const float* verts, const int nverts,
								const float minh, const float maxh, unsigned char area)
{
	if (m_volumeCount >= MAX_VOLUMES) return;
	ConvexVolume* vol = &m_volumes[m_volumeCount++];
	memset(vol, 0, sizeof(ConvexVolume));
	memcpy(vol->verts, verts, sizeof(float)*3*nverts);
	vol->hmin = minh;
	vol->hmax = maxh;
	vol->nverts = nverts;
	vol->area = area;
}

void InputGeom::deleteConvexVolume(int i)
{
	m_volumeCount--;
	m_volumes[i] = m_volumes[m_volumeCount];
}

// sp ep与p 在y轴上相差一定距离,判定sp->p->ep向量是否与mesh可走区域相交
// 向量与mesh相交 则p为可选取的输入点， 否则为不可选取的点.
bool InputGeom::IsValidate(const float* p)
{
	float sp[3], ep[3], hittime;
	float dp[3] = {0, 100, 0};

	rcVcopy(sp, p); 
	rcVcopy(ep, p);

	rcVadd(sp, sp, dp);
	rcVsub(ep, ep, dp);

	return raycastMesh(sp, ep, hittime);
}


bool InputGeom::raycastHit(const float *p, float *raycastHit)
{
	float sp[3], ep[3], hitTime;
	float dp[3] = {0, 100, 0}; 

	rcVcopy(sp, p); 
	rcVcopy(ep, p); 

	rcVadd(sp, sp, dp);
	rcVsub(ep, ep, dp);

	bool hit = raycastMesh(sp, ep, hitTime);

	if (hit)
	{   
		raycastHit[0] = sp[0] + (ep[0] - sp[0]) * hitTime;
		raycastHit[1] = sp[1] + (ep[1] - sp[1]) * hitTime + 0.5;
		raycastHit[2] = sp[2] + (ep[2] - sp[2]) * hitTime;
		return true;
	}   
	return false;
}


