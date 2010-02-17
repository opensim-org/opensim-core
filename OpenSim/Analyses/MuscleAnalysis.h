#ifndef _MuscleAnalysis_h_
#define _MuscleAnalysis_h_
// MuscleAnalysis.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS: Katherine Holzbaur, Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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
 * @author Katherine Holzbaur, Frank C. Anderson
 * @version 1.0
 */
class OSIMANALYSES_API MuscleAnalysis : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
public:
#ifndef SWIG
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
	/// @endcond

#endif
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
	// Copy constrctor and virtual copy 
	MuscleAnalysis(const MuscleAnalysis &aObject);
	virtual Object* copy() const;
	virtual ~MuscleAnalysis();
private:
	void setNull();
	void setupProperties();
	void constructDescription();
	void allocateStorageObjects();
	void updateStorageObjects();
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
	Storage* getPennationAngleStorage() const { return _pennationAngleStore; }
	Storage* getMuscleTendonLengthStorage() const { return _lengthStore; }
	Storage* getFiberLengthStorage() const { return _fiberLengthStore; }
	Storage* getNormalizedFiberLengthStorage() const { return _normalizedFiberLengthStore; }
	Storage* getTendonLegthStorage() const { return _tendonLengthStore; }
	Storage* getForceStorage() const { return _forceStore; }
	Storage* getFiberForceStorage() const { return _fiberForceStore; }
	Storage* getActiveFiberForceStorage() const { return _activeFiberForceStore; }
	Storage* getPassiveFiberForceStorage() const { return _passiveFiberForceStore; }
	Storage* getActiveFiberForceAlongTendonStorage() const { return _activeFiberForceAlongTendonStore; }
	Storage* getPassiveFiberForceAlongTendonStorage() const { return _passiveFiberForceAlongTendonStore; }
	void setMuscles(Array<std::string>& aMuscles);
	void setCoordinates(Array<std::string>& aCoordinates);

	void setComputeMoments(bool aTrueFalse) {
		_computeMoments = aTrueFalse;
	}
	bool getComputeMoments() const {
		return _computeMoments;
	}
	//const ArrayPtrs<StorageCoordinatePair>& getMomentArmStorageArray() const { return _momentArmStorageArray; }

	//--------------------------------------------------------------------------
	// ANALYSIS
	//--------------------------------------------------------------------------
#ifndef SWIG
	virtual int
        begin(const SimTK::State& s );
    virtual int
        step(const SimTK::State& s, int setNumber );
    virtual int
        end(const SimTK::State& s );
protected:
    virtual int
        record(const SimTK::State& s );
#endif	
	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
public:
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

	OPENSIM_DECLARE_DERIVED(MuscleAnalysis,Analysis)
//=============================================================================
};	// END of class MuscleAnalysis

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __MuscleAnalysis_h__
