#ifndef __InverseKinematicsTool_h__
#define __InverseKinematicsTool_h__
// InverseKinematicsTool.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2010, Stanford University. All rights reserved. 
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