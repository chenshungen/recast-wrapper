#ifndef CONVEXVOLUMEBUILDER_H
#define CONVEXVOLUMEBUILDER_H

#include "Navigation.h"

// Tool to create convex volumess for InputGeom

class ConvexVolumeBuilder
{
	Navigation* m_nav;
	int m_areaType;
	float m_polyOffset;
	float m_boxHeight;
	float m_boxDescent;
	
	static const int MAX_PTS = 12;
	float m_pts[MAX_PTS*3];
	int m_npts;
	int m_hull[MAX_PTS];
	int m_nhull;
	
public:
	ConvexVolumeBuilder();
	virtual void init(Navigation* nav);
	virtual void reset();

	void AddConvexVolume(int npts,  const float* pts, int areaType);
	void DelConvexVolume(const float* p);
};

#endif // CONVEXVOLUME_H
