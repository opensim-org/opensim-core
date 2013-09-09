#ifndef OPENSIM_EXTERNAL_LOADS_H_
#define OPENSIM_EXTERNAL_LOADS_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ExternalLoads.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Ayman Habib                                          *
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


// INCLUDES
#include "ModelComponentSet.h"
#include "ExternalForce.h"
#include "OpenSim/Common/PropertyStr.h"
#include "OpenSim/Common/PropertyDbl.h"

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * A convenience class for managing ExternaForce(s) to be applied to a model.
 * This includes creating instances and manipulating the data source
 * of inividual ExternalForces so that they satsify conditions imposed
 * by particular Tools. For example, ForwardTool, CMC/RRA, achieve better
 * tracking (slower divergence) if the ground reaction forces are applied
 * to a point that is expressed in the foot frame according to "ideal"
 * kinematics. ExternalLoads provides convenience methdods to perform this
 * "mapping" which is beyond the scope of an individual ExternalForce, but is
 * too much detail to have each Tool implement.
 *
 * An indiviudal ExternalForce has a property for its data source name, but 
 * under the management of ExternalLoads, the data source identified by
 * ExternalLoads is used to set the data source on each ExternalForce. 
 * If multiple data sources are required for different groups of external forces
 * then use multiple ExternalLoads.
 *
 * @authors Ajay Seth, Ayman Habib 
 */

//=============================================================================
class OSIMSIMULATION_API ExternalLoads 
:   public ModelComponentSet<ExternalForce> {
OpenSim_DECLARE_CONCRETE_OBJECT(ExternalLoads, 
                                ModelComponentSet<ExternalForce>);

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Data source for all forces in this ExternalLoads, where individual 
	 *  external forces identify which subsets of the data they will acceess.*/
	PropertyStr _dataFileNameProp;
	std::string &_dataFileName;
	/** Name of the file containing the model kinematics corresponding to the
	external loads. */
	PropertyStr _externalLoadsModelKinematicsFileNameProp;
	std::string &_externalLoadsModelKinematicsFileName;
	/** Low-pass cut-off frequency for filtering the model kinematics corresponding
	to the external loads. A negative value results in no filtering.
	The default value is -1.0, so no filtering. */
	PropertyDbl _lowpassCutoffFrequencyForLoadKinematicsProp;
	double &_lowpassCutoffFrequencyForLoadKinematics;

private:
	/* If point of applications for external forces must be re-expressed
	   then build new storages to be assigned to the individual ExternalForces
	   with the transformed point data. Hang-on to them so we can delete them. */
	ArrayPtrs<Storage> _storages;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ExternalLoads();
	ExternalLoads(Model& model);
	ExternalLoads(Model& model, const std::string &aFileName, bool aUpdateFromXMLNode = true)  SWIG_DECLARE_EXCEPTION;
	ExternalLoads(const ExternalLoads &aExternalLoads);
	virtual ~ExternalLoads();

	void copyData(const ExternalLoads &otherExternalLoads);

	/** Override of the default implementation to account for versioning. */
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);

	// Connect all ExternalForces inside this ExternalLoads collection to
    // their Model. Overrides ModelComponentSet method.
	void invokeConnectToModel(Model& aModel) OVERRIDE_11;

	const Model& getModel() const	// get around wrapping issue where exposing the method in ModelComponentSet is problematic
    {
        return	*_model;
    }

	const std::string& getDataFileName() const { return _dataFileName;};
	void setDataFileName(const std::string& aNewFile) { _dataFileName = aNewFile; };

	const std::string &getExternalLoadsModelKinematicsFileName() const { return _externalLoadsModelKinematicsFileName; }
	void setExternalLoadsModelKinematicsFileName(const std::string &aFileName) { _externalLoadsModelKinematicsFileName = aFileName; }
	double getLowpassCutoffFrequencyForLoadKinematics() const { return _lowpassCutoffFrequencyForLoadKinematics; }
	void setLowpassCutoffFrequencyForLoadKinematics(double aLowpassCutoffFrequency) { _lowpassCutoffFrequencyForLoadKinematics = aLowpassCutoffFrequency; }

	void transformPointsExpressedInGroundToAppliedBodies(const Storage &kinematics, double startTime = -SimTK::Infinity, double endTime = SimTK::Infinity);
	ExternalForce* transformPointExpressedInGroundToAppliedBody(const ExternalForce &exForce, const Storage &kinematics, double startTime, double endTime);

private:
	void setNull();
	void setupSerializedMembers();
	std::string createIdentifier(OpenSim::Array<std::string>&oldFunctionNames, const Array<std::string>& labels);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	ExternalLoads& operator=(const ExternalLoads &otherExternalLoads);
#endif


//=============================================================================
};	// END of class ExternalLoads
//=============================================================================
//=============================================================================

} // end of namespace OpenSim


#endif // OPENSIM_EXTERNAL_LOADS_H_


