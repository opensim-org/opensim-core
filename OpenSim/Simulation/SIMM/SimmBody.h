#ifndef _SimmBody_h_
#define _SimmBody_h_

// SimmBody.h
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


// INCLUDE
#include <iostream>
#include <string>
#include <math.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/Set.h>
#include <OpenSim/Tools/VisibleObject.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/PropertyObjArray.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/XMLDocument.h>
#include "SimmMarker.h"

namespace OpenSim { 

class SimmKinematicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM body segment. simmBodies can contain bones,
 * as well as mass properties.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API SimmBody : public Object  
{

//=============================================================================
// DATA
//=============================================================================
public:
#ifndef SWIG
	typedef struct
	{
		bool used;
		int timesSplit;
		double massFactor;
		bool skippable;
	} sdfastBodyInfo;

	sdfastBodyInfo _sdfastInfo;
#endif
protected:
	PropertyDbl _massProp;
	double &_mass;

	PropertyDblArray _massCenterProp;
	Array<double> &_massCenter;

	PropertyDblArray _inertiaProp;
	Array<double> &_inertia;

	PropertyObjArray _bonesProp;
	ArrayPtrs<VisibleObject> &_bones;

	PropertyObjArray _markersProp;
	ArrayPtrs<SimmMarker> &_markers;

	/* For holding cumulative scale factors, which are needed if
	 * a SIMM joint file is written (so SIMM knows how much to
	 * scale the bones.
	 */
	double _scaleFactor[3];

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmBody();
	SimmBody(DOMElement *aElement);
	SimmBody(const SimmBody &aBody);
	virtual ~SimmBody();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

#ifndef SWIG
	SimmBody& operator=(const SimmBody &aBody);
#endif
   void SimmBody::copyData(const SimmBody &aBody);

   void setup(SimmKinematicsEngine* aEngine);

	double getMass() const { return _mass; }
	void getMassCenter(double vec[3]) { vec[0] = _massCenter[0]; vec[1] = _massCenter[1]; vec[2] = _massCenter[2]; }
	const Array<double>& getInertia() { return _inertia; }
	ArrayPtrs<VisibleObject>& getBones() const { return _bones; }
	int getNumMarkers() { return _markers.getSize(); }
	SimmMarker* getMarker(int index) const;
	int deleteAllMarkers();
	void deleteMarker(const SimmMarker* aMarker);
	int deleteUnusedMarkers(const Array<std::string>& aMarkerNames);
	void scale(Array<double>& aScaleFactors, bool aPreserveMassDist = false);
	void scaleInertialProperties(Array<double>& aScaleFactors);

	void addMarker(SimmMarker* aMarker);
	void writeSIMM(std::ofstream& out) const;
	void writeMarkers(std::ofstream& out) const;

	void peteTest() const;

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmBody

//=============================================================================
//=============================================================================

typedef RDSIMULATION_API OpenSim::ArrayPtrs<OpenSim::SimmBody> SimmBodyArray;

}; //namespace
#endif // __SimmBody_h__


