/*---------------------------------------------------------------------------
Copyright (C) GeoLib.
This code is used under license from GeoLib (www.geolib.co.uk). This or
any modified versions of this cannot be resold to any other party.
---------------------------------------------------------------------------*/

/**--------------------------------------------------------------------------<BR>
\file StdAfx.h
\brief File used for all commonly used includes within the library.
<P>---------------------------------------------------------------------------*/

#ifndef _GEOLIB_STDAFX_H 
#define _GEOLIB_STDAFX_H

#ifdef _WIN32
#	ifdef GeoLib_EXPORTS
#		define GeoLib_API __declspec(dllexport)
#	else
#		define GeoLib_API  __declspec(dllimport)
#	endif
#else
#	define GeoLib_API
#endif // WIN32


#include <cassert>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <algorithm>
#include "Constants.h"

class C2DBase;
typedef std::vector<C2DBase*> C2DBaseData;

#endif
