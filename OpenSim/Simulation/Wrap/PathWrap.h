#ifndef OPENSIM_PATH_WRAP_H_
#define OPENSIM_PATH_WRAP_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  PathWrap.h                            *
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyIntArray.h>
#include "PathWrapPoint.h"
#include "WrapResult.h"

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class WrapObject;
class SimbodyEngine;
class GeometryPath;

//=============================================================================
//=============================================================================
/**
 * A class implementing an instance of muscle wrapping. That is, it is owned
 * by a particular muscle, and contains parameters for wrapping that muscle
 * over a particular wrap object.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API PathWrap : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(PathWrap, Object);

//=============================================================================
// DATA
//=============================================================================
public:

	enum WrapMethod
	{
		hybrid,
		midpoint,
		axial
	};

protected:
	PropertyStr _wrapObjectNameProp;
    std::string &_wrapObjectName;

	PropertyStr _methodNameProp;   // currently used only for ellipsoid wrapping
	std::string& _methodName;
	WrapMethod _method;

    PropertyIntArray _rangeProp;
    Array<int> &_range;

	WrapObject* _wrapObject;
	GeometryPath* _path;

	WrapResult _previousWrap;  // results from previous wrapping

    PathWrapPoint _wrapPoints[2]; // the two muscle points created when the muscle wraps

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PathWrap();
	PathWrap(const PathWrap& aPathWrap);
	~PathWrap();

#ifndef SWIG
	PathWrap& operator=(const PathWrap& aPathWrap);
	void connectToModelAndPath(const Model& aModel, GeometryPath& aPath);
	void setStartPoint( const SimTK::State& s, int aIndex);
	void setEndPoint( const SimTK::State& s, int aIndex);
#endif
    void copyData(const PathWrap& aPathWrap);
	int getStartPoint() const { return _range[0]; }
	int getEndPoint() const { return _range[1]; }
	const std::string& getWrapObjectName() const { return _wrapObjectName; }
	WrapObject* getWrapObject() const { return _wrapObject; }
	void setWrapObject(WrapObject& aWrapObject);
	PathWrapPoint& getWrapPoint(int aIndex);
	WrapMethod getMethod() const { return _method; }
	void setMethod(WrapMethod aMethod);
	const std::string& getMethodName() const { return _methodName; }
	GeometryPath* getPath() const { return _path; }

	const WrapResult& getPreviousWrap() const { return _previousWrap; }
	void setPreviousWrap(const WrapResult& aWrapResult);
	void resetPreviousWrap();

protected:
	void setupProperties();

private:
	void setNull();
//=============================================================================
};	// END of class PathWrap
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_PATH_WRAP_H_


