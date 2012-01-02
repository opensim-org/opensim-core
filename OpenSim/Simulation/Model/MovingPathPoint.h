#ifndef __MovingPathPoint_h__
#define __MovingPathPoint_h__

// MovingPathPoint.h
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
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Simulation/Model/PathPoint.h>
#include "SimTKcommon.h"
#include "SimTKsimbody.h"

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

//=============================================================================
//=============================================================================
/**
 * A class implementing a moving muscle point, which is a muscle point that
 * moves in a body's reference frame as a function of a coordinate.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API MovingPathPoint : public PathPoint  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	PropertyObjPtr<Function> _xLocationProp;
	Function* &_xLocation;

	PropertyStr _xCoordinateNameProp;
   std::string &_xCoordinateName;

	const Coordinate* _xCoordinate;

	PropertyObjPtr<Function> _yLocationProp;
	Function* &_yLocation;

	PropertyStr _yCoordinateNameProp;
   std::string &_yCoordinateName;

	const Coordinate* _yCoordinate;

	PropertyObjPtr<Function> _zLocationProp;
	Function* &_zLocation;

	PropertyStr _zCoordinateNameProp;
   std::string &_zCoordinateName;

	const Coordinate* _zCoordinate;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	MovingPathPoint();
	MovingPathPoint(const MovingPathPoint &aPoint);
	virtual ~MovingPathPoint();
	virtual Object* copy() const;

#ifndef SWIG
	MovingPathPoint& operator=(const MovingPathPoint &aPoint);
#endif
   void copyData(const MovingPathPoint &aPoint);
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber);
	virtual void init(const PathPoint& aPoint);

	const Coordinate* getXCoordinate() const { return _xCoordinate; }
	const Coordinate* getYCoordinate() const { return _yCoordinate; }
	const Coordinate* getZCoordinate() const { return _zCoordinate; }
#ifndef SWIG
	void setXCoordinate( const SimTK::State& s, Coordinate& aCoordinate);
	void setYCoordinate( const SimTK::State& s, Coordinate& aCoordinate);
	void setZCoordinate( const SimTK::State& s, Coordinate& aCoordinate);
	virtual bool isActive(const SimTK::State& s) const { return true; }
	virtual void setup(const Model& aModel, GeometryPath& aPath);
	virtual void update(const SimTK::State& s);
	virtual void getVelocity(const SimTK::State& s, SimTK::Vec3& aVelocity);
#endif
	const std::string& getXCoordinateName() const { return _xCoordinateName; }
	const std::string& getYCoordinateName() const { return _yCoordinateName; }
	const std::string& getZCoordinateName() const { return _zCoordinateName; }
	virtual Function* getXFunction() const { return _xLocation; }
	virtual Function* getYFunction() const { return _yLocation; }
	virtual Function* getZFunction() const { return _zLocation; }
#ifndef SWIG
	void setXFunction( const SimTK::State& s, Function& aFunction);
	void setYFunction( const SimTK::State& s, Function& aFunction);
	void setZFunction( const SimTK::State& s, Function& aFunction);
#endif
   virtual void scale(const SimTK::State& s, const SimTK::Vec3& aScaleFactors);


	OPENSIM_DECLARE_DERIVED(MovingPathPoint, PathPoint);
private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class MovingPathPoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MovingPathPoint_h__


