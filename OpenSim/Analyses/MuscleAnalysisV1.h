#ifndef _MuscleAnalysisV1_h_
#define _MuscleAnalysisV1_h_
// MuscleAnalysisV1.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS: Matthew Millard, Katherine Holzbaur, Frank C. Anderson
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

/*
* 2011 09 23
* This function was updated to also store data on
*
* dlce
* fv
* fal
* fpe
* fse
*
* To make it possible to reproduce the dimensionless muscle curves that are being used in simulation
* for debugging purposes
*/

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimAnalysesDLL.h"
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Common/NaturalCubicSpline.h>


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
class OSIMANALYSES_API MuscleAnalysisV1 : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleAnalysisV1, Analysis);

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


	//MM
	Storage *_dactStore; /** Rate Change of activation */
	Storage *_actStore; /** Activation storage */
	Storage *_falStore; /** Active force-length storage (normalized) */
	Storage *_fseStore; /** Tendon force-length storage (normalized)*/
	Storage *_fpeStore; /** Muscle passive force length storage (normalized)*/
	Storage *_fvStore; /** Force multiplying factor */
	Storage *_fvVmaxStore; /** Vmax... internally used by fv */
	Storage *_tlStore; /** Tendon length (normalized)*/
	Storage *_lceStore; /** Contractile element length storage (normalized)*/
	Storage *_dlceStore; /** Contractile element velocity storage (normalized)*/
	Storage *_uStore; /** Excitation storage */
	Storage *_caStore;//Cosine(pennationAngle)

	Storage *_fsePEStore;	//Potential energy stored in the tendon
	Storage *_fpePEStore;	//Potential energy stored in the muscle
	Storage *_musclePWRStore; //Work done by the muscle fibers.
	Storage *_muscleFStore; //Work done by the muscle fibers.
	Storage *_muscleVStore; //Work done by the muscle fibers.
	Storage *_mclTdnKEPEWStore;//Muscle Tendon: KE+PE-W

	//MM	Spline functions used to interpolate the curves that are used as Gold Standards for 
	//		the force-velocity, active-force-length, passive muscle force, and tendon stiffness
	NaturalCubicSpline *_ncs_stdfal;
	NaturalCubicSpline *_ncs_stdfv;
	NaturalCubicSpline *_ncs_stdfpe;
	NaturalCubicSpline *_ncs_stdfse;

	Storage *_fvErrStore;//Difference between model's fv, and gold standard fv for a given velocity
	Storage *_falErrStore;//Difference between model's fal, and gold standard fal for a given velocity
	Storage *_fpeErrStore;//Difference between model's fpe, and gold standard fpe for a given velocity
	Storage *_fseErrStore;//Difference between model's fse, and gold standard fse for a given velocity
	Storage *_mcltenErrStore;//Store the normalized squared sum of the previous errors.


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
	MuscleAnalysisV1(Model *aModel=0);
	MuscleAnalysisV1(const std::string &aFileName);
	MuscleAnalysisV1(const MuscleAnalysisV1 &aObject);
	virtual ~MuscleAnalysisV1();

private:
	void setNull();
	void setupProperties();
	void constructDescription();
	void allocateStorageObjects();
	void updateStorageObjects();
	void constructColumnLabels();
	void setStandardMuscleCurves(); //MM
	NaturalCubicSpline* get1DSpline(const std::string &aFileName); //MM
	double get1DSplineValue(const NaturalCubicSpline *aSpline, double xval); //MM 

public:
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	MuscleAnalysisV1& operator=(const MuscleAnalysisV1 &aMuscleAnalysisV1);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	virtual void setModel(Model& aModel);
	void setStorageCapacityIncrements(int aIncrement);

	Storage* getfvStorage() const { return _fvStore; }

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
        begin( SimTK::State& s );
    virtual int
        step(const SimTK::State& s, int setNumber );
    virtual int
        end( SimTK::State& s );
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
//=============================================================================
};	// END of class MuscleAnalysisV1

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __MuscleAnalysisV1_h__
