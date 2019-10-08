#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include <values.h>
#include <new>
#include "InputGeom.h"
#include "Navigation.h"
#include "Recast.h"
#include "DetourCrowd.h"
#include "DetourAssert.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourCommon.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "PathFinding.h"
#include "CrowdManager.h"
#include "ConvexVolumeBuilder.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"
#include "fastlz.h"

#ifdef WIN32
#   define snprintf _snprintf
#endif

// This value specifies how many layers (or "floors") each navmesh tile is expected to have.
static const int EXPECTED_LAYERS_PER_TILE = 4;

static bool isectSegAABB(const float* sp, const float* sq,
						 const float* amin, const float* amax,
						 float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;
	
	float d[3];
	rcVsub(d, sq, sp);
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
			if (t1 > t2) rcSwap(t1, t2);
			// Compute the intersection of slab intersections intervals
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			// Exit with no collision as soon as slab intersection becomes empty
			if (tmin > tmax) return false;
		}
	}
	
	return true;
}

//static int calcLayerBufferSize(const int gridWidth, const int gridHeight)
//{
	//const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	//const int gridSize = gridWidth * gridHeight;
	//return headerSize + gridSize*4;
//}


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

struct LinearAllocator : public dtTileCacheAlloc
{
	unsigned char* buffer;
	size_t capacity;
	size_t top;
	size_t high;
	
	LinearAllocator(const size_t cap) : buffer(0), capacity(0), top(0), high(0)
	{
		resize(cap);
	}
	
	~LinearAllocator()
	{
		dtFree(buffer);
	}

	void resize(const size_t cap)
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
	
	virtual void* alloc(const size_t size)
	{
		if (!buffer)
			return 0;
		if (top+size > capacity)
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

struct MeshProcess : public dtTileCacheMeshProcess
{

	InputGeomPtr m_geom;

	inline MeshProcess() : m_geom(0)
	{
	}

	inline void init(InputGeomPtr geom)
	{
		m_geom = geom;
	}
	
	virtual void process(struct dtNavMeshCreateParams* params, unsigned char* polyAreas, unsigned short* polyFlags)
	{
		// Update poly flags from areas.
		for (int i = 0; i < params->polyCount; ++i)
		{
			if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
				polyAreas[i] = POLYAREA_GROUND;

			if (polyAreas[i] == POLYAREA_GROUND ||
				polyAreas[i] == POLYAREA_GRASS ||
				polyAreas[i] == POLYAREA_ROAD)
			{
				polyFlags[i] = POLYFLAGS_WALK;
			}
			else if (polyAreas[i] == POLYAREA_WATER)
			{
				polyFlags[i] = POLYFLAGS_SWIM;
			}
			else if (polyAreas[i] == POLYAREA_DOOR)
			{
				polyFlags[i] = POLYFLAGS_WALK | POLYFLAGS_DOOR;
			}
		}

		// Pass in off-mesh connections.
		if (m_geom)
		{
			params->offMeshConVerts = m_geom->getOffMeshConnectionVerts();
			params->offMeshConRad = m_geom->getOffMeshConnectionRads();
			params->offMeshConDir = m_geom->getOffMeshConnectionDirs();
			params->offMeshConAreas = m_geom->getOffMeshConnectionAreas();
			params->offMeshConFlags = m_geom->getOffMeshConnectionFlags();
			params->offMeshConUserID = m_geom->getOffMeshConnectionId();
			params->offMeshConCount = m_geom->getOffMeshConnectionCount();	
		}
	}
};

static const int MAX_LAYERS = 32;

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
		delete [] triareas;
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

int Navigation::rasterizeTileLayers(const int tx, const int ty, const rcConfig& cfg, TileCacheData* tiles, const int maxTiles)
{
	if (!m_geom || !m_geom->getMesh() || !m_geom->getChunkyMesh())
	{
		m_ctx->log(RC_LOG_ERROR, "buildTile: Input mesh is not specified.");
		return 0;
	}
	
	FastLZCompressor comp;
	RasterizationContext rc;
	
	const float* verts = m_geom->getMesh()->getVerts();
	const int nverts = m_geom->getMesh()->getVertCount();
	const rcChunkyTriMesh* chunkyMesh = m_geom->getChunkyMesh();
	
	// Tile bounds.
	const float tcs = cfg.tileSize * cfg.cs;
	
	rcConfig tcfg;
	memcpy(&tcfg, &cfg, sizeof(tcfg));

	tcfg.bmin[0] = cfg.bmin[0] + tx*tcs;
	tcfg.bmin[1] = cfg.bmin[1];
	tcfg.bmin[2] = cfg.bmin[2] + ty*tcs;
	tcfg.bmax[0] = cfg.bmin[0] + (tx+1)*tcs;
	tcfg.bmax[1] = cfg.bmax[1];
	tcfg.bmax[2] = cfg.bmin[2] + (ty+1)*tcs;
	tcfg.bmin[0] -= tcfg.borderSize*tcfg.cs;
	tcfg.bmin[2] -= tcfg.borderSize*tcfg.cs;
	tcfg.bmax[0] += tcfg.borderSize*tcfg.cs;
	tcfg.bmax[2] += tcfg.borderSize*tcfg.cs;
	
	// Allocate voxel heightfield where we rasterize our input data to.
	rc.solid = rcAllocHeightfield();
	if (!rc.solid)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return 0;
	}
	if (!rcCreateHeightfield(m_ctx, *rc.solid, tcfg.width, tcfg.height, tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return 0;
	}
	
	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	rc.triareas = new unsigned char[chunkyMesh->maxTrisPerChunk];
	if (!rc.triareas)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", chunkyMesh->maxTrisPerChunk);
		return 0;
	}
	
	float tbmin[2], tbmax[2];
	tbmin[0] = tcfg.bmin[0];
	tbmin[1] = tcfg.bmin[2];
	tbmax[0] = tcfg.bmax[0];
	tbmax[1] = tcfg.bmax[2];
	int cid[512];// TODO: Make grow when returning too many items.
	const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
	if (!ncid)
	{
		return 0; // empty
	}
	
	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
		const int* tris = &chunkyMesh->tris[node.i*3];
		const int ntris = node.n;
		
		memset(rc.triareas, 0, ntris*sizeof(unsigned char));
		rcMarkWalkableTriangles(m_ctx, tcfg.walkableSlopeAngle,
								verts, nverts, tris, ntris, rc.triareas);
		
		if (!rcRasterizeTriangles(m_ctx, verts, nverts, tris, rc.triareas, ntris, *rc.solid, tcfg.walkableClimb))
			return 0;
	}
	
	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (m_filterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(m_ctx, tcfg.walkableClimb, *rc.solid);
	if (m_filterLedgeSpans)
		rcFilterLedgeSpans(m_ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid);
	if (m_filterWalkableLowHeightSpans)
		rcFilterWalkableLowHeightSpans(m_ctx, tcfg.walkableHeight, *rc.solid);
	
	
	rc.chf = rcAllocCompactHeightfield();
	if (!rc.chf)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return 0;
	}
	if (!rcBuildCompactHeightfield(m_ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid, *rc.chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return 0;
	}
	
	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(m_ctx, tcfg.walkableRadius, *rc.chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return 0;
	}
	
	// (Optional) Mark areas.
	const ConvexVolume* vols = m_geom->getConvexVolumes();
	for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
	{
		rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts,
							 vols[i].hmin, vols[i].hmax,
							 (unsigned char)vols[i].area, *rc.chf);
	}
	
	rc.lset = rcAllocHeightfieldLayerSet();
	if (!rc.lset)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'lset'.");
		return 0;
	}
	if (!rcBuildHeightfieldLayers(m_ctx, *rc.chf, tcfg.borderSize, tcfg.walkableHeight, *rc.lset))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build heighfield layers.");
		return 0;
	}
	
	rc.ntiles = 0;
	for (int i = 0; i < rcMin(rc.lset->nlayers, MAX_LAYERS); ++i)
	{
		TileCacheData* tile = &rc.tiles[rc.ntiles++];
		const rcHeightfieldLayer* layer = &rc.lset->layers[i];
		
		// Store header
		dtTileCacheLayerHeader header;
		header.magic = DT_TILECACHE_MAGIC;
		header.version = DT_TILECACHE_VERSION;
		
		// Tile layer location in the navmesh.
		header.tx = tx;
		header.ty = ty;
		header.tlayer = i;
		dtVcopy(header.bmin, layer->bmin);
		dtVcopy(header.bmax, layer->bmax);
		
		// Tile info.
		header.width = (unsigned char)layer->width;
		header.height = (unsigned char)layer->height;
		header.minx = (unsigned char)layer->minx;
		header.maxx = (unsigned char)layer->maxx;
		header.miny = (unsigned char)layer->miny;
		header.maxy = (unsigned char)layer->maxy;
		header.hmin = (unsigned short)layer->hmin;
		header.hmax = (unsigned short)layer->hmax;

		dtStatus status = dtBuildTileCacheLayer(&comp, &header, layer->heights, layer->areas, layer->cons,
												&tile->data, &tile->dataSize);
		if (dtStatusFailed(status))
		{
			return 0;
		}
	}

	// Transfer ownsership of tile data from build context to the caller.
	int n = 0;
	for (int i = 0; i < rcMin(rc.ntiles, maxTiles); ++i)
	{
		tiles[n++] = rc.tiles[i];
		rc.tiles[i].data = 0;
		rc.tiles[i].dataSize = 0;
	}
	
	return n;
}

dtObstacleRef hitTestObstacle(const dtTileCache* tc, const float* sp, const float* sq)
{
	float tmin = FLT_MAX;
	const dtTileCacheObstacle* obmin = 0;
	for (int i = 0; i < tc->getObstacleCount(); ++i)
	{
		const dtTileCacheObstacle* ob = tc->getObstacle(i);
		if (ob->state == DT_OBSTACLE_EMPTY)
			continue;
		
		float bmin[3], bmax[3], t0,t1;
		tc->getObstacleBounds(ob, bmin,bmax);
		
		if (isectSegAABB(sp,sq, bmin,bmax, t0,t1))
		{
			if (t0 < tmin)
			{
				tmin = t0;
				obmin = ob;
			}
		}
	}
	return tc->getObstacleRef(obmin);
}



Navigation::Navigation() : 
	m_geom(0),
	m_navMesh(0),
	m_navQuery(0),
	m_crowd(0),
	m_filterLowHangingObstacles(true),
	m_filterLedgeSpans(true),
	m_filterWalkableLowHeightSpans(true),
	m_ctx(0),
	m_tileCache(0),
	m_cacheBuildTimeMs(0),
	m_cacheCompressedSize(0),
	m_cacheRawSize(0),
	m_cacheLayerCount(0),
	m_cacheBuildMemUsage(0),
	m_maxTiles(0),
	m_maxPolysPerTile(0),
	m_tileSize(48)
{
	resetCommonSettings();

	m_navQuery = dtAllocNavMeshQuery();
	m_crowd = dtAllocCrowd();
	
	m_talloc = new LinearAllocator(96000);
	m_tcomp = new FastLZCompressor;
	m_tmproc = new MeshProcess;
	m_pathfind = new PathFinding();
	m_crowdMgr = new CrowdManager();
}

Navigation::~Navigation()
{
	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;
	dtFreeTileCache(m_tileCache);
}

void Navigation::handleSettings()
{
	if (m_geom)
	{
		const float* bmin = m_geom->getNavMeshBoundsMin();
		const float* bmax = m_geom->getNavMeshBoundsMax();
		int gw = 0, gh = 0;
		rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
		const int ts = (int)m_tileSize;
		const int tw = (gw + ts-1) / ts;
		const int th = (gh + ts-1) / ts;

		// Max tiles and max polys affect how the tile IDs are caculated.
		// There are 22 bits available for identifying a tile and a polygon.
		int tileBits = rcMin((int)dtIlog2(dtNextPow2(tw*th*EXPECTED_LAYERS_PER_TILE)), 14);
		if (tileBits > 14) tileBits = 14;
		int polyBits = 22 - tileBits;
		m_maxTiles = 1 << tileBits;
		m_maxPolysPerTile = 1 << polyBits;
	}
	else
	{
		m_maxTiles = 0;
		m_maxPolysPerTile = 0;
	}
}

void Navigation::resetCommonSettings()
{
	m_cellSize = 0.33f;
	m_cellHeight = 0.2f;
	m_agentHeight = 2.0f;
	m_agentRadius = 0.5f;
	m_agentMaxClimb = 1.5f;
	m_agentMaxSlope = 75.0f;
	m_regionMinSize = 2;
	m_regionMergeSize = 20;
	m_edgeMaxLen = 12.0f;
	m_edgeMaxError = 1.3f;
	m_vertsPerPoly = 6.0f;
	m_detailSampleDist = 6.0f;
	m_detailSampleMaxError = 1.0f;
	m_partitionType = PARTITION_WATERSHED;
}

void Navigation::ChangeMesh(InputGeomPtr geom)
{
	m_geom = geom;

	dtFreeTileCache(m_tileCache);
	m_tileCache = 0;

	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;

	m_pathfind->Reset();
	m_pathfind->Init(this);
	m_crowdMgr->init(this);
	m_tmproc->init(m_geom);
}

bool Navigation::addTempObstacle(const float* pos, const float radius)
{
	if (!m_tileCache)
		return false;
	float p[3];
	dtVcopy(p, pos);
	p[1] -= 0.5f;
	dtStatus status = m_tileCache->addObstacle(p, radius, 2.0f, 0);
	return status == DT_SUCCESS;
}

// sq obstacle pos
void Navigation::removeTempObstacle(/*const float* sp,*/ const float* sq)
{
	if (!m_tileCache)
		return;
	float ray[3] = {0, 1, 0};
	float sp[3];
	dtVadd(sp, sq, ray);
	dtObstacleRef ref = hitTestObstacle(m_tileCache, sp, sq);
	m_tileCache->removeObstacle(ref);
}

void Navigation::clearAllTempObstacles()
{
	if (!m_tileCache)
		return;
	for (int i = 0; i < m_tileCache->getObstacleCount(); ++i)
	{
		const dtTileCacheObstacle* ob = m_tileCache->getObstacle(i);
		if (ob->state == DT_OBSTACLE_EMPTY) continue;
		m_tileCache->removeObstacle(m_tileCache->getObstacleRef(ob));
	}
}

void Navigation::addOffMeshConnection(const float* s, const float* e, bool bidir)
{
	const unsigned char area = POLYAREA_JUMP;
	const unsigned short flags = POLYFLAGS_JUMP; 
	m_geom->addOffMeshConnection(s, e, getAgentRadius(), bidir ? 1 : 0, area, flags);
	ValidateOffMeshConnection(s);
}

void Navigation::deleteOffMeshConnection(const float* pos)
{
	// Delete
	// Find nearest link end-point
	float nearestDist = FLT_MAX;
	int nearestIndex = -1;
	const float* verts = m_geom->getOffMeshConnectionVerts();
	for (int i = 0; i < m_geom->getOffMeshConnectionCount()*2; ++i)
	{
		const float* v = &verts[i*3];
		float d = rcVdistSqr(pos, v);
		if (d < nearestDist)
		{
			nearestDist = d;
			nearestIndex = i/2; // Each link has two vertices.
		}
	}

	// If end point close enough, delete it.
	if (nearestIndex != -1 && sqrtf(nearestDist) < getAgentRadius())
	{
		m_geom->deleteOffMeshConnection(nearestIndex);
		ValidateOffMeshConnection(pos);
	}
}

void Navigation::DoUpdate(const float dt)
{
	if (!m_navMesh)
		return;
	if (!m_tileCache)
		return;

	m_tileCache->update(dt, m_navMesh);

	m_crowdMgr->DoUpdate(dt);
}

void Navigation::updateTileCache()
{
	if (!m_tileCache)
		return;
	m_tileCache->update(0.05f, m_navMesh);
}

void Navigation::getTilePos(const float* pos, int& tx, int& ty)
{
	if (!m_geom) return;
	
	const float* bmin = m_geom->getNavMeshBoundsMin();
	
	const float ts = m_tileSize*m_cellSize;
	tx = (int)((pos[0] - bmin[0]) / ts);
	ty = (int)((pos[2] - bmin[2]) / ts);
}



static const int TILECACHESET_MAGIC = 'T'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'TSET';
static const int TILECACHESET_VERSION = 1;

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

void Navigation::saveAll(const char* path)
{
	if (!m_tileCache) return;
	
	FILE* fp = fopen(path, "wb");
	if (!fp)
		return;
	
	// Store header.
	TileCacheSetHeader header;
	header.magic = TILECACHESET_MAGIC;
	header.version = TILECACHESET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < m_tileCache->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = m_tileCache->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.cacheParams, m_tileCache->getParams(), sizeof(dtTileCacheParams));
	memcpy(&header.meshParams, m_navMesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(TileCacheSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < m_tileCache->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = m_tileCache->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		TileCacheTileHeader tileHeader;
		tileHeader.tileRef = m_tileCache->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);
}

bool Navigation::Load(const char* path)
{
	dtFreeNavMesh(m_navMesh);
	dtFreeTileCache(m_tileCache);
	if (!loadAll(path))
		return false;
	m_navQuery->init(m_navMesh, 2048);
	m_pathfind->Init(this);
	m_crowdMgr->init(this);
	return true;
}

bool Navigation::loadAll(const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp) 
		return false;
	
	// Read header.
	TileCacheSetHeader header;
	size_t headerReadReturnCode = fread(&header, sizeof(TileCacheSetHeader), 1, fp);
	if( headerReadReturnCode != 1)
	{
		// Error or early EOF
		fclose(fp);
		return false;
	}
	if (header.magic != TILECACHESET_MAGIC)
	{
		fclose(fp);
		return false;
	}
	if (header.version != TILECACHESET_VERSION)
	{
		fclose(fp);
		return false;
	}
	
	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		fclose(fp);
		return false;
	}
	dtStatus status = m_navMesh->init(&header.meshParams);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return false;
	}

	m_tileCache = dtAllocTileCache();
	if (!m_tileCache)
	{
		fclose(fp);
		return false;
	}
	status = m_tileCache->init(&header.cacheParams, m_talloc, m_tcomp, m_tmproc);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return false;
	}
		
	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		TileCacheTileHeader tileHeader;
		size_t tileHeaderReadReturnCode = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if( tileHeaderReadReturnCode != 1)
		{
			// Error or early EOF
			fclose(fp);
			return false;
		}
		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		size_t tileDataReadReturnCode = fread(data, tileHeader.dataSize, 1, fp);
		if( tileDataReadReturnCode != 1)
		{
			// Error or early EOF
			dtFree(data);
			fclose(fp);
			return false;
		}
		
		dtCompressedTileRef tile = 0;
		dtStatus addTileStatus = m_tileCache->addTile(data, tileHeader.dataSize, DT_COMPRESSEDTILE_FREE_DATA, &tile);
		if (dtStatusFailed(addTileStatus))
		{
			dtFree(data);
		}

		if (tile)
			m_tileCache->buildNavMeshTile(tile, m_navMesh);
	}

	
	fclose(fp);

	return true;
}

// 需要设置限制， 不能Rebuild过多的tile 性能消耗会很大
void Navigation::RebuildTileAt(const float* pts, int npts)
{
	int minX=INT_MAX, minY=INT_MAX, maxX=INT_MIN, maxY=INT_MIN;
	for (int i = 0; i < npts; i++)
	{
		int tx=0, ty=0;
		getTilePos(&pts[3*i], tx, ty);
		minX = dtMin(tx, minX);
		maxX = dtMax(tx, maxX);
		minY = dtMin(ty, minY);
		maxY = dtMax(ty, maxY);
	}
	// Init cache
	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	// Generation params.
	rcConfig cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.cs = m_cellSize;
	cfg.ch = m_cellHeight;
	cfg.walkableSlopeAngle = m_agentMaxSlope;
	cfg.walkableHeight = (int)ceilf(m_agentHeight / cfg.ch);
	cfg.walkableClimb = (int)floorf(m_agentMaxClimb / cfg.ch);
	cfg.walkableRadius = (int)ceilf(m_agentRadius / cfg.cs);
	cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	cfg.maxSimplificationError = m_edgeMaxError;
	cfg.minRegionArea = (int)rcSqr(m_regionMinSize);		// Note: area = size*size
	cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// Note: area = size*size
	cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
	cfg.tileSize = (int)m_tileSize;
	cfg.borderSize = cfg.walkableRadius + 3; // Reserve enough padding.
	cfg.width = cfg.tileSize + cfg.borderSize*2;
	cfg.height = cfg.tileSize + cfg.borderSize*2;
	cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;
	rcVcopy(cfg.bmin, bmin);
	rcVcopy(cfg.bmax, bmax);

	for (int tx = minX; tx <= maxX; tx++)
	{
		for (int ty = minY; ty <= maxY; ty++)
		{
			dtCompressedTileRef mtiles[MAX_LAYERS];

			int ntiles = m_tileCache->getTilesAt(tx,ty,mtiles,MAX_LAYERS);

			for (int i = 0; i < ntiles; ++i)
			{
				dtStatus status = m_tileCache->removeTile(mtiles[i], 0, 0);
				if (dtStatusFailed(status))
				{
					continue;
				}
			}

			TileCacheData tiles[MAX_LAYERS];
			ntiles = rasterizeTileLayers(tx, ty, cfg, tiles, MAX_LAYERS);
			for (int i = 0; i < ntiles; ++i)
			{
				TileCacheData* tile = &tiles[i];
				dtStatus status = m_tileCache->addTile(tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);
				if (dtStatusFailed(status))
				{
					dtFree(tile->data);
					tile->data = 0;
					continue;
				}
			}

			m_tileCache->buildNavMeshTilesAt(tx,ty, m_navMesh);
		}
	}
}

void Navigation::ValidateOffMeshConnection(const float* pos)
{
	int tx=0, ty=0;
	getTilePos(pos, tx, ty);
	m_tileCache->buildNavMeshTilesAt(tx,ty, m_navMesh);
}

float* Navigation::FindSmoothPath(const float* start, const float* end, const float stepSize, int& pathSize)
{
	return m_pathfind->FindSmoothPath(start, end, stepSize, pathSize);
}

float* Navigation::FindPath(const float* start, const float* end, int& point_num)
{
	return m_pathfind->FindPath(start, end, point_num);
}

float* Navigation::MakeRandomPoints(int num)
{
	return m_pathfind->MakeRandomPoints(num);
}

float* Navigation::MakeRandomPointsAround(const float* pos, const float radius, int num)
{
	return m_pathfind->MakeRandomPointsAround(pos, radius, num);
}

void Navigation::MakeRandomPoint(float* pos)
{
	m_pathfind->MakeRandomPoint(pos);
}

bool Navigation::MakeRandomPointInRing(const float *center, const float inradius, const float outradius, float *pos)
{
	return m_pathfind->MakeRandomPointInRing(center, inradius, outradius, pos);
}

int Navigation::CreateAgent(const float* pos, const float radius)
{
	return m_crowdMgr->CreateAgent(pos, radius);
}

void Navigation::AgentMoveTo(float* target, int idx, bool vel)
{
	return m_crowdMgr->AgentMoveTo(target, idx, vel);
}

bool Navigation::GetAgentPosition(int idx, float* p)
{
	return m_crowdMgr->GetAgentPosition(idx, p);
}

void Navigation::ResetMoveOp(int idx)
{
	m_crowdMgr->ResetMoveOp(idx);
}

bool Navigation::GetAgentVel(int idx, float* p)
{
	return m_crowdMgr->GetAgentVel(idx, p);
}

void Navigation::SetAgentMaxSpeed(int idx, float speed)
{
	m_crowdMgr->SetAgentMaxSpeed(idx, speed);
}

float Navigation::GetAgentMaxSpeed(int idx)
{
	return m_crowdMgr->GetAgentMaxSpeed(idx);
}

const dtCrowdAgent* Navigation::GetAgent(int idx)
{
	return m_crowdMgr->GetAgent(idx);
}

void Navigation::RemoveAgent(const int idx)
{
	m_crowdMgr->removeAgent(idx);
}

void Navigation::ResetAgent(int idx, const float* pos)
{
	m_crowdMgr->ResetAgent(idx, pos);
}

void Navigation::getBounds(float* boundsMin, float* boundsMax)
{
	const float* bMin = m_geom->getMeshBoundsMin();
	const float* bMax = m_geom->getMeshBoundsMax();
	dtVcopy(boundsMin, bMin);
	dtVcopy(boundsMax, bMax);
}

float Navigation::GetDistanceToClosestWall(const float* pos, float* tp) 
{
	return m_pathfind->GetDistanceToClosestWall(pos, tp); 
}


Navigation* CreateNavSDK(std::string navmesh, std::string tilecache)
{
	Navigation* nav = new Navigation;
	auto geomIter = InputGeom::m_inputGeoms.find(navmesh);
	if (geomIter == InputGeom::m_inputGeoms.end())
		return nullptr;

	nav->ChangeMesh(InputGeom::m_inputGeoms[navmesh]);
	nav->handleSettings();
	if (tilecache == "tilecache.bin")
	{
		std::string::size_type idx = navmesh.rfind(".", navmesh.length());
		tilecache = navmesh.substr(0, idx) + ".bin";
	}
	if (!nav->Load(tilecache.c_str()))
	{
		delete nav;
		nav = nullptr;
	}

	return nav;
}



