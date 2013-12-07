#ifndef OPENSIM_MOVING_PATH_POINT_H_
#define OPENSIM_MOVING_PATH_POINT_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  MovingPathPoint.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


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
class OSIMSIMULATION_API MovingPathPoint : public PathPoint {
OpenSim_DECLARE_CONCRETE_OBJECT(MovingPathPoint, PathPoint);

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

    // Override methods from PathPoint.
	bool isActive(const SimTK::State& s) const OVERRIDE_11 { return true; }
	void connectToModelAndPath(const Model& aModel, GeometryPath& aPath) 
                                                                OVERRIDE_11;
	void update(const SimTK::State& s) OVERRIDE_11;
	void getVelocity(const SimTK::State& s, SimTK::Vec3& aVelocity) OVERRIDE_11;
#endif
	SimTK::Vec3 getdPointdQ(const SimTK::State& s) const OVERRIDE_11; 

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

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class MovingPathPoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_MOVING_PATH_POINT_H_


