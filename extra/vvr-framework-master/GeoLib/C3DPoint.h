/*---------------------------------------------------------------------------
Copyright (C) GeoLib.
This code is used under license from GeoLib (www.geolib.co.uk). This or
any modified versions of this cannot be resold to any other party.
---------------------------------------------------------------------------*/


/**--------------------------------------------------------------------------<BR>
\file 3DPoint.h
\brief Header file for a 3D point class.

Header file for a simple 3D point class.

\class C3DPoint
\brief 3D Point class.

3D Point class.
<P>---------------------------------------------------------------------------*/
#ifndef _GEOLIB_C3DPOINT_H 
#define _GEOLIB_C3DPOINT_H

#include "MemoryPool.h"

class GeoLib_API C3DPoint
{
public:
	_MEMORY_POOL_DECLARATION

	/// Constructor.
	C3DPoint(void);
	/// Destructor.
	~C3DPoint(void);
	/// Distance to another point.
	double Distance(const C3DPoint& Other) const;

	// x co-ordinate.
	double x;
	// y co-ordinate.
	double y;
	// z co-ordinate.
	double z;

};

#endif
