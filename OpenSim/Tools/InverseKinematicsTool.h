#ifndef __InverseKinematicsTool_h__
#define __InverseKinematicsTool_h__
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  InverseKinematicsTool.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include "Tool.h"

#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif

namespace OpenSim {

class Model;
class IKTaskSet;
class Storage;
//=============================================================================
//=============================================================================
/**
 * A Tool that performs an Inverse Kinematics analysis with a given model.
 * Inverse kinematics is the solution of internal coordinates that poses
 * the model such that the landmark locations (markers), affixed to the model,
 * minimize the weighted least-squares error with observations of markers 
 * in spatial coordinates. Observations of coordinates can also be included.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMTOOLS_API InverseKinematicsTool: public Tool {
OpenSim_DECLARE_CONCRETE_OBJECT(InverseKinematicsTool, Tool);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:
	
	/** Pointer to the model being investigated. */
	Model *_model;

	/** Name of the xml file used to deserialize or construct a model. */
	PropertyStr _modelFileNameProp;
	std::string &_modelFileName;

	/** The relative weight of the constraints. Infinity is a strictly enforced constraint. 
		Any other non-zero positive scalar is the penalty factor for constraint violations. */
	PropertyDbl _constraintWeightProp;
	double &_constraintWeight;

	/** The accuracy of the solution in absolute terms, i.e. the number of significant digits
	    to which the solution can be trusted. */
	PropertyDbl _accuracyProp;
	double &_accuracy;

	// Markers and coordinates to be matched and their respective weightings
	PropertyObj _ikTaskSetProp;
	IKTaskSet &_ikTaskSet;

	// name of marker file that contains marker locations for IK solving
	PropertyStr _markerFileNameProp;
	std::string &_markerFileName;

	// name of storage file that contains coordinate values for IK solving
	PropertyStr _coordinateFileNameProp;
	std::string &_coordinateFileName;

	// range of frames to solve in marker file, specified by time
	PropertyDblArray _timeRangeProp;
	Array<double> &_timeRange;

	// flag if inverse kinematics should report marker errors
	PropertyBool _reportErrorsProp;
	bool &_reportErrors;

	// name of motion file with inverse kinematics solution
	PropertyStr _outputMotionFileNameProp;
	std::string &_outputMotionFileName;

	// flag indicating whether or not to resulting marker locations
	PropertyBool _reportMarkerLocationsProp;
	bool &_reportMarkerLocations;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~InverseKinematicsTool();
	InverseKinematicsTool();
	InverseKinematicsTool(const std::string &aFileName, bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;
	InverseKinematicsTool(const InverseKinematicsTool &aObject);

	/* Register types to be used when reading an InverseKinematicsTool object from xml file. */
	static void registerTypes();
	/* Handle reading older formats/Versioning */
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);

	//---- Setters and getters for various attributes
	void setModel(Model& aModel) { _model = &aModel; };
	void setStartTime(double d) { _timeRange[0] = d; };
	double getStartTime() const {return  _timeRange[0]; };

	void setEndTime(double d) { _timeRange[1] = d; };
	double getEndTime() const {return  _timeRange[1]; };

	void setMarkerDataFileName(const std::string& markerDataFileName) { _markerFileName=markerDataFileName;};
	const std::string& getMarkerDataFileName() const { return  _markerFileName;};

	void setCoordinateFileName(const std::string& coordDataFileName) { _coordinateFileName=coordDataFileName;};
	const std::string& getCoordinateFileName() const { return  _coordinateFileName;};
    
	//const OpenSim::Storage& getOutputStorage() const;
private:
	void setNull();
	void setupProperties();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	InverseKinematicsTool&
		operator=(const InverseKinematicsTool &aInverseKinematicsTool);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setOutputMotionFileName(const std::string aOutputMotionFileName) {
		_outputMotionFileName = aOutputMotionFileName;
	}
	std::string getOutputMotionFileName() { return _outputMotionFileName;}
	IKTaskSet& getIKTaskSet() { return _ikTaskSet; }

	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual bool run() SWIG_DECLARE_EXCEPTION;


//=============================================================================
};	// END of class InverseKinematicsTool
//=============================================================================
} // namespace

#endif // __InverseKinematicsTool_h__