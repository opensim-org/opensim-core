#ifndef __PathWrapPoint_h__
#define __PathWrapPoint_h__

// PathWrapPoint.h
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


// INCLUDE
#include <iostream>
#include <string>
#include <math.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Simulation/Model/PathPoint.h>

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class Coordinate;
class Model;
class GeometryPath;
class SimbodyEngine;
class WrapObject;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM muscle via point, which is a muscle point that
 * is active only for a specified range of a coordinate.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API PathWrapPoint : public PathPoint {
OpenSim_DECLARE_CONCRETE_OBJECT(PathWrapPoint, PathPoint);

//=============================================================================
// DATA
//=============================================================================
private:
	Array<SimTK::Vec3> _wrapPath; // points defining muscle path on surface of wrap object
   double _wrapPathLength; // length of _wrapPath

	WrapObject* _wrapObject; // the wrap object this point is on

protected:

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PathWrapPoint();
	PathWrapPoint(const PathWrapPoint &aPoint);
	virtual ~PathWrapPoint();

	void copyData(const PathWrapPoint &aPoint);

#ifndef SWIG
	PathWrapPoint& operator=(const PathWrapPoint &aPoint);
	virtual void setup(const Model& aModel, GeometryPath& aPath);
#endif

	Array<SimTK::Vec3>& getWrapPath() { return _wrapPath; }
	double getWrapLength() const { return _wrapPathLength; }
	void setWrapLength(double aLength) { _wrapPathLength = aLength; }
	virtual WrapObject* getWrapObject() const { return _wrapObject; }
	void setWrapObject(WrapObject* aWrapObject) { _wrapObject = aWrapObject; }

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class PathWrapPoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __PathWrapPoint_h__


