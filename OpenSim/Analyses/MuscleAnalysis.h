#ifndef _MuscleAnalysis_h_
#define _MuscleAnalysis_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  MuscleAnalysis.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Matthew Millard, Katherine R. S. Holzbaur,           *
 *            Frank C. Anderson                                               *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimAnalysesDLL.h"
#include <OpenSim/Simulation/Model/Muscle.h>


#ifdef SWIG
	#ifdef OSIMANALYSES_API
		#undef OSIMANALYSES_API
		#define OSIMANALYSES_API
	#endif
#endif



namespace OpenSim { 


//=============================================================================
//=============================================================================
/**
 * A class for recording and computting basic quantities (length, shortening
 * velocity, tendon length, ...) for muscles during a simulation.
 *
 * @author Ajay Seth, Matthew Millard, Katherine Holzbaur, Frank C. Anderson 
 * @version 1.0
 */
class OSIMANALYSES_API MuscleAnalysis : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleAnalysis, Analysis);
#ifndef SWIG
//=============================================================================
// DATA
//=============================================================================
public:
	// STRUCT FOR PAIRING MOMENT ARM STORAGE OBJECTS WITH THEIR
	// ASSOCIATE GENERALIZED COORDINATE

	typedef struct {
		Coordinate *q;
		Storage *momentArmStore;
		Storage *momentStore;
	}  
// Excluding this from Doxygen until it has better documentation! -Sam Hamner
    /// @cond

	StorageCoordinatePair;
#endif
	/// @endcond
private:

	/** List of muscles for which to compute moment arms. */
	PropertyStrArray _muscleListProp;

	/** List of generalized coordinates for which to compute moment arms. */
	PropertyStrArray _coordinateListProp;

	/** Compute moments and moment arms. */
	PropertyBool _computeMomentsProp;

	/** Pennation angle storage. */
	Storage *_pennationAngleStore;
	/** Muscle-tendon length storage. */
	Storage *_lengthStore;
	/** Fiber length storage. */
	Storage *_fiberLengthStore;
	/** Normalized fiber length storage. */
	Storage *_normalizedFiberLengthStore;
	/** Tendon length storage. */
	Storage *_tendonLengthStore;

	/** Lengthening velocity of the muscle fibers. */
	Storage *_fiberVelocityStore;
	/** Normalized lengthening velocity of the muscle fibers. */
	Storage *_normFiberVelocityStore;
	/** Angular velocity of the muscle fibers. */
	Storage *_pennationAngularVelocityStore;

	/** Force applied by the muscle. */
	Storage *_forceStore;
	/** Force in the muscle fibers. */
	Storage *_fiberForceStore;
	/** Active force in the muscle fibers. */
	Storage *_activeFiberForceStore;
	/** Passive force in the muscle fibers. */
	Storage *_passiveFiberForceStore;
	/** Active force in the muscle fibers along tendon. */
	Storage *_activeFiberForceAlongTendonStore;
	/** Passive force in the muscle fibers along tendon. */
	Storage *_passiveFiberForceAlongTendonStore;

	/** Fiber power */
	Storage *_fiberActivePowerStore;
    Storage *_fiberPassivePowerStore;
	/** Tendon power */
	Storage *_tendonPowerStore;
	/** Muscle actuator power */
	Storage *_musclePowerStore;

	// FOR MOMENT ARMS AND MOMENTS----------------
	/** Work array for holding the list of muscles.  This array */
	Array<std::string> _muscleList;

	/** Work array for holding the list of coordinates. */
	Array<std::string> _coordinateList;

	bool _computeMoments;
#ifndef SWIG
	/** Array of active storage and coordinate pairs. */
	ArrayPtrs<StorageCoordinatePair> _momentArmStorageArray;
#endif
	/** Array of active muscles. */
	ArrayPtrs<Muscle> _muscleArray;

//=============================================================================
// METHODS
//=============================================================================
public:
	MuscleAnalysis(Model *aModel=0);
	MuscleAnalysis(const std::string &aFileName);
	MuscleAnalysis(const MuscleAnalysis &aObject);
	virtual ~MuscleAnalysis();

private:
	void setNull();
	void setupProperties();
	void constructDescription();
	void constructColumnLabels();

public:
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	MuscleAnalysis& operator=(const MuscleAnalysis &aMuscleAnalysis);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	virtual void setModel(Model& aModel);
	void setStorageCapacityIncrements(int aIncrement);

    Storage* getPennationAngleStorage() const { 
        return _pennationAngleStore; }
	Storage* getMuscleTendonLengthStorage() const { 
        return _lengthStore; }
	Storage* getFiberLengthStorage() const { 
        return _fiberLengthStore; }
	Storage* getNormalizedFiberLengthStorage() const { 
        return _normalizedFiberLengthStore; }
	Storage* getTendonLengthStorage() const { 
        return _tendonLengthStore; }

    Storage* getFiberVelocityStorage() const { 
        return _fiberVelocityStore; }
    Storage* getNormalizedFiberVelocityStorage() const { 
        return _normFiberVelocityStore; }
    Storage* getPennationAngularVelocityStorage() const { 
        return _pennationAngularVelocityStore; }

	Storage* getForceStorage() const { 
        return _forceStore; }
	Storage* getFiberForceStorage() const { 
        return _fiberForceStore; }
	Storage* getActiveFiberForceStorage() const { 
        return _activeFiberForceStore; }
	Storage* getPassiveFiberForceStorage() const { 
        return _passiveFiberForceStore; }
	Storage* getActiveFiberForceAlongTendonStorage() const { 
        return _activeFiberForceAlongTendonStore; }
	Storage* getPassiveFiberForceAlongTendonStorage() const { 
        return _passiveFiberForceAlongTendonStore; }
	
    Storage* getFiberActivePowerStorage() const { 
        return _fiberActivePowerStore; }
    Storage* getFiberPassivePowerStorage() const { 
        return _fiberPassivePowerStore; }
    Storage* getTendonPowerStorage() const { 
        return _tendonPowerStore; }
    Storage* getMusclePowerStorage() const { 
        return _musclePowerStore; }

    void setMuscles(Array<std::string>& aMuscles);
	void setCoordinates(Array<std::string>& aCoordinates);

	void setComputeMoments(bool aTrueFalse) {
		_computeMoments = aTrueFalse;
	}
	bool getComputeMoments() const {
		return _computeMoments;
	}
#ifndef SWIG
	const ArrayPtrs<StorageCoordinatePair>& getMomentArmStorageArray() const { return _momentArmStorageArray; }
#endif
	//--------------------------------------------------------------------------
	// ANALYSIS
	//--------------------------------------------------------------------------
	virtual int
        begin( SimTK::State& s );
    virtual int
        step(const SimTK::State& s, int setNumber );
    virtual int
        end( SimTK::State& s );
protected:
    virtual int
        record(const SimTK::State& s );
	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
public:
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");
	/** 
     * Intended for use only by GUI that holds one MuscleAnalysis and keeps changing attributes to generate various plots
     * For all other use cases, the code handles the allocation/deallocation of resources internally.
     */
 	void allocateStorageObjects();
//=============================================================================
};	// END of class MuscleAnalysis

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __MuscleAnalysis_h__
