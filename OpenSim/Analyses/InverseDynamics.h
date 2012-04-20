#ifndef _InverseDynamics_h_
#define _InverseDynamics_h_
// InverseDynamics.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Eran Guendelman
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
#include "osimAnalysesDLL.h"
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <SimTKcommon.h>


//=============================================================================
//=============================================================================
/**
 */
namespace OpenSim { 

class Model;
class ForceSet;

/** @cond **/ // hide from Doxygen

/**
 * A class for performing and recording Inverse Dynamics forces/moments
 * on a motion trajectory.
 *
 * @author Eran
 */
class OSIMANALYSES_API InverseDynamics : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(InverseDynamics, Analysis);

//=============================================================================
// DATA
//=============================================================================
private:
	int _numCoordinateActuators;
protected:
	/** Use force set from model. */
	PropertyBool _useModelForceSetProp;
	bool &_useModelForceSet;

	Storage *_storage;
	GCVSplineSet _statesSplineSet;

	Array<double> _dydt;
	Array<int> _accelerationIndices;

	bool _ownsForceSet;
	ForceSet *_forceSet;

	SimTK::Matrix _performanceMatrix;
	SimTK::Vector _performanceVector;
	SimTK::Matrix _constraintMatrix;
	SimTK::Vector _constraintVector;
	SimTK::Vector _lapackWork;

	Model *_modelWorkingCopy;

//=============================================================================
// METHODS
//=============================================================================
public:
	InverseDynamics(Model *aModel=0);
	InverseDynamics(const InverseDynamics &aObject);
	virtual ~InverseDynamics();

    //--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	InverseDynamics& operator=(const InverseDynamics &aInverseDynamics);
#endif
private:
	void setNull();
	void setupProperties();
	void constructDescription();
	void constructColumnLabels();
	void allocateStorage();
	void deleteStorage();
	void computeAcceleration(SimTK::State& s, double *aF,double *rAccel) const;

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setStorageCapacityIncrements(int aIncrement);
	Storage* getStorage();

	bool getUseModelForceSet() { return _useModelForceSet; }
	void setUseModelForceSet(bool aUseModelForceSet) { _useModelForceSet = aUseModelForceSet; }

	virtual void setModel(Model& aModel);
	//--------------------------------------------------------------------------
	// ANALYSIS
	//--------------------------------------------------------------------------
#ifndef SWIG
	virtual int
        begin(SimTK::State& s );
    virtual int
        step(const SimTK::State& s, int setNumber );
    virtual int
        end(SimTK::State& s );
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
};	// END of class InverseDynamics

/** @endcond **/

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __InverseDynamics_h__
