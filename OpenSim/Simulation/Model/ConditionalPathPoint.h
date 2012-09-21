#ifndef OPENSIM_CONDITIONAL_PATH_POINT_H_
#define OPENSIM_CONDITIONAL_PATH_POINT_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  ConditionalPathPoint.h                      *
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
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/Storage.h>
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

//=============================================================================
//=============================================================================
/**
 * A class implementing a conditional path point, which is a point that
 * is active only for a specified range of a coordinate.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API ConditionalPathPoint : public PathPoint {
OpenSim_DECLARE_CONCRETE_OBJECT(ConditionalPathPoint, PathPoint);

//=============================================================================
// DATA
//=============================================================================
protected:
    PropertyDblArray _rangeProp;
    Array<double> &_range;

    PropertyStr _coordinateNameProp;
    std::string &_coordinateName;

	const Coordinate* _coordinate;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ConditionalPathPoint();
	ConditionalPathPoint(const ConditionalPathPoint &aPoint);
	virtual ~ConditionalPathPoint();

#ifndef SWIG
	ConditionalPathPoint& operator=(const ConditionalPathPoint &aPoint);
#endif
    void copyData(const ConditionalPathPoint &aPoint);
	virtual void init(const PathPoint& aPoint);
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);

	Array<double>& getRange() const { return _range; }
	const Coordinate* getCoordinate() const { return _coordinate; }
	const std::string& getCoordinateName() const { return _coordinateName; }
#ifndef SWIG
	void setCoordinate(const SimTK::State& s, Coordinate& aCoordinate);
	void setRangeMin( const SimTK::State& s, double aMin);
	void setRangeMax( const SimTK::State& s, double aMax);

    // Override PathPoint methods.
	bool isActive(const SimTK::State& s) const OVERRIDE_11;
	void connectToModelAndPath(const Model& aModel, GeometryPath& aPath) 
                                                                OVERRIDE_11;
#endif
private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class ConditionalPathPoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_CONDITIONAL_PATH_POINT_H_


