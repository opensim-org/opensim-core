#ifndef __ExternalLoads_h__
#define __ExternalLoads_h__

// ExternalLoads.h
// Author: Ajay Seth, Ayman Habib 
/*
 * Copyright (c)  2009, Stanford University. All rights reserved. 
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


// INCLUDES
#include "ModelComponentSet.h"
#include "ExternalForce.h"
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
 * "mapping" which is beyong the scope of an individual ExternalForce, but is
 * too much detail to have each Tool implement.
 *
 * An indiviudal ExternalForce has a property for its data source name, but 
 * under the management of ExternalLoads, the data source identified by
 * ExternalLoads is used to set the data source on each ExternalForce. 
 * If multiple data sources are required for different groups of external forces
 * then use multiple ExternalLoads.
 *
 * @authors Ajay Seth, Ayman Habib 
 * @version 1.0
 */

//=============================================================================
class OSIMSIMULATION_API ExternalLoads : public ModelComponentSet<ExternalForce>
{

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
	OpenSim::PropertyStr _externalLoadsModelKinematicsFileNameProp;
	std::string &_externalLoadsModelKinematicsFileName;
	/** Low-pass cut-off frequency for filtering the model kinematics corresponding
	to the external loads. A negative value results in no filtering.
	The default value is -1.0, so no filtering. */
	OpenSim::PropertyDbl _lowpassCutoffFrequencyForLoadKinematicsProp;
	double &_lowpassCutoffFrequencyForLoadKinematics;

private:
	/* If point of applications for external forces must be re-expressed
	   then build new storages to be assigned to the individual ExternalForces
	   with the transformed point data. Hang-on to them so we can delete them. */
	SimTK::Array_<Storage *> _storages;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ExternalLoads();
	ExternalLoads(Model& model);
	ExternalLoads(Model& model, const std::string &aFileName, bool aUpdateFromXMLNode = true);
	ExternalLoads(const ExternalLoads &aExternalLoads);
	virtual ~ExternalLoads();
	virtual Object* copy() const;
	void copyData(const ExternalLoads &otherExternalLoads);

	/** Override of the default implementation to account for versioning. */
	virtual void updateFromXMLNode();

	// setup all ExternalForces inside this ExternalLoads collection 
	virtual void setup(Model& aModel);

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


#endif // __ExternalLoads_h__


