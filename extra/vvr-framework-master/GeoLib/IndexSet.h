/*---------------------------------------------------------------------------
Copyright (C) GeoLib.
This code is used under license from GeoLib (www.geolib.co.uk). This or
any modified versions of this cannot be resold to any other party.
---------------------------------------------------------------------------*/


/**--------------------------------------------------------------------------<BR>
\file IndexSet.h
\brief Declaration file for the CIndexSet class. 

\class CIndexSet
\brief Class which is used to store a set of indexes in a simple array. 
<P>---------------------------------------------------------------------------*/

#include "StdAfx.h"

#ifndef _GEOLIB_CINDEXSET_H 
#define _GEOLIB_CINDEXSET_H

class CSetData;

class GeoLib_API CIndexSet
{
public:
	/// Constructor
	CIndexSet(void);
	/// Destructor
	~CIndexSet(void);
	/// Adds an index
	void Add(unsigned int nIndex);
	/// Retrives an index in the array.
	unsigned int& operator[](unsigned int nIndex) const;
	/// Gets the size.
	unsigned int size(void) const;
private:
	/// The data.
	CSetData* m_Data;
};

#endif
