#ifndef __ExternalLoads_h__
#define __ExternalLoads_h__

// ExternalLoads.h
// Author: Ayman Habib 
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Set.h>

#include "Force.h"
#include "Actuator.h"
#include "Muscle.h"
#include "ModelComponentSet.h"
#include "SimTKsimbody.h"

namespace OpenSim {

class Model;
class VectorGCVSplineR1R3;
class GCVSpline;
//=============================================================================
//=============================================================================
/**
 * A class for holding and managing ExternalLoads to be applied to a model
 *
 * @authors Ayman Habib 
 * @version 1.0
 */

//=============================================================================
class OSIMSIMULATION_API ExternalLoads : public ForceSet
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** In case the Forces in the set are loaded from file, the filename goes here
	 * and the column names go into individual functions.
	 */
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
	void copyData(const ExternalLoads &aAbsExternalLoads);
	void createForcesFromFile(const std::string& datafileName,
								Array<std::string>& startForceColumns,
								Array<int>& columnCount, 
								Array<std::string>& bodyNames);

	const std::string& getDataFileName() const { return _dataFileName;};
	void setDataFileName(const std::string& aNewFile) { _dataFileName = aNewFile; };
	const std::string &getExternalLoadsModelKinematicsFileName() const { return _externalLoadsModelKinematicsFileName; }
	void setExternalLoadsModelKinematicsFileName(const std::string &aFileName) { _externalLoadsModelKinematicsFileName = aFileName; }
	double getLowpassCutoffFrequencyForLoadKinematics() const { return _lowpassCutoffFrequencyForLoadKinematics; }
	void setLowpassCutoffFrequencyForLoadKinematics(double aLowpassCutoffFrequency) { _lowpassCutoffFrequencyForLoadKinematics = aLowpassCutoffFrequency; }

private:
	void setNull();
	void setupSerializedMembers();
	void copyForce(Force* aFrom, Force* aTo);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	ExternalLoads& operator=(const ExternalLoads &aSet);
#endif
     void computePointFunctions(SimTK::State& s, 
                                double startTime,
                                double endTime,
                                const Body& body,
                                const Storage& aQStore,
                                const Storage& aUStore,
                                VectorGCVSplineR1R3& aPGlobal,
                                GCVSpline*& xfunc, 
                                GCVSpline*& yfunc, 
                                GCVSpline*& zfunc);
	 void computeFunctions(SimTK::State& s, 
                                double startTime,
                                double endTime, 
								const Storage& kineticsStore, 
								Storage* qStore=NULL, 
								Storage* uStore=NULL);
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	virtual void setup(Model& aModel);

//=============================================================================
};	// END of class ExternalLoads
//=============================================================================
//=============================================================================

} // end of namespace OpenSim


#endif // __ExternalLoads_h__


