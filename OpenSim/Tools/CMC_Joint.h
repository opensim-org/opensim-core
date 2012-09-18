#ifndef CMCJoint_h__
#define CMCJoint_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  CMC_Joint.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 * Contributor(s): Frank C. Anderson                                          *
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
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//============================================================================
// INCLUDE
//============================================================================
#include "osimToolsDLL.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "CMC_Task.h"

namespace OpenSim {

class Coordinate;

//=============================================================================
//=============================================================================
/**
 * A class for specifying the tracking task for a joint.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API CMC_Joint : public CMC_Task {
OpenSim_DECLARE_CONCRETE_OBJECT(CMC_Joint, CMC_Task);

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Name of the generalized coordinate to be tracked. */
	PropertyStr _propCoordinateName;
	/** Error limit on the coordinate. */
	PropertyDbl _propLimit;

	// REFERENCES
	std::string &_coordinateName;
	double &_limit;

	// Work Variables
	Coordinate *_q;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	CMC_Joint(const std::string &aCoordinateName = "");
	CMC_Joint(const CMC_Joint &aTask);
	virtual ~CMC_Joint();

private:
	void setNull();
	void setupProperties();
	void copyData(const CMC_Joint &aTask);
	void updateWorkVariables();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	CMC_Joint& operator=(const CMC_Joint &aTask);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	virtual void setModel(Model& aModel);
	void setCoordinateName(const std::string &aName);
	std::string getCoordinateName() const;
	double getLimit() const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeErrors(const SimTK::State& s, double aT);
	virtual void computeDesiredAccelerations(const SimTK::State& s, double aT);
	virtual void computeDesiredAccelerations(const SimTK::State& s, double aTI,double aTF);
	virtual void computeAccelerations(const SimTK::State& s );

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);


//=============================================================================
};	// END of class CMC_Joint
//=============================================================================
//=============================================================================

}; // end namespace

#endif // CMCJoint_h__


