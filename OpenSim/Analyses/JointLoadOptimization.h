#ifndef _JointLoadOptimization_h_
#define _JointLoadOptimization_h_
// JointLoadOptimization.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Matt DeMers
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
#include <OpenSim/Simulation/JointReactionReference.h>
#include <SimTKcommon.h>


//=============================================================================
//=============================================================================
/**
 */
namespace OpenSim { 

class Model;
class ForceSet;

/**
 * This class implements static optimization to compute Muscle Forces and 
 * activations. It includes extra terms in the cost function for minimizing
 * or maximizing joint forces and moments.
 *
 * @author Matt DeMers
 */
class OSIMANALYSES_API JointLoadOptimization : public Analysis 
{
	OPENSIM_DECLARE_DERIVED(JointLoadOptimization, Analysis);
//=============================================================================
// DATA
//=============================================================================
private:
	int _numCoordinateActuators;
	Array<std::string> _jointForceLabels;
protected:
	/** Use force set from model. */

	PropertyBool _useModelForceSetProp;
	bool &_useModelForceSet;

	PropertyDbl _activationExponentProp;
	double &_activationExponent;

	PropertyBool _useMusclePhysiologyProp;
	bool	&_useMusclePhysiology;

	// Added by Matt DeMers to optimize joint forces and moments
	//---------------------------------------------------------

	PropertyDbl _convergenceCriterionProp;
	double &_convergenceCriterion;

	PropertyInt _printLevelProp;
	int &_printLevel;

	PropertyInt _maxIterationsProp;
	int &_maxIterations;

	PropertyDbl _optimizerDXProp;
	double &_optimizerDX;

	PropertyObj _jointTaskSetProp;
	Set<JointReactionReference> &_jointTaskSet;

	PropertyStr _optimizerAlgorithmProp;
	std::string &_optimizerAlgorithm;

	PropertyDbl _objectiveScaleFactorProp;
	double &_objectiveScaleFactor;

	PropertyDbl _jointTermScaleProp;
	double &_jointTermScale;

	//---------------------------------------------------------

	Storage *_activationStorage;
	Storage *_forceStorage;
	Storage *_jointForceStorage;
	GCVSplineSet _statesSplineSet;

	Array<int> _accelerationIndices;
	Array<int> _optimizedJointsIndices;

	SimTK::Vector _parameters;

	bool _ownsForceSet;
	ForceSet* _forceSet;

	//double _optimizerDX;
	//std::string _optimizerAlgorithm;
	//int _printLevel;
	//double _convergenceCriterion;
	//int _maxIterations;

	Model *_modelWorkingCopy;

//=============================================================================
// METHODS
//=============================================================================
public:
	JointLoadOptimization(Model *aModel=0);
	// Copy constrctor and virtual copy 
	JointLoadOptimization(const JointLoadOptimization &aObject);
	virtual Object* copy() const;
	virtual ~JointLoadOptimization();
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	JointLoadOptimization& operator=(const JointLoadOptimization &aJointLoadOptimization);
#endif
private:
	void setNull();
	void setupProperties();
	void constructDescription();
	void constructColumnLabels();
	void allocateStorage();
	void deleteStorage();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setStorageCapacityIncrements(int aIncrement);
	Storage* getActivationStorage();
	Storage* getForceStorage();
	Storage* getJointForceStorage();

	bool getUseModelForceSet() { return _useModelForceSet; }
	void setUseModelForceSet(bool aUseModelActuatorSet) { _useModelForceSet = aUseModelActuatorSet; }

	virtual void setModel(Model& aModel);
	void setActivationExponent(const double aExponent) { _activationExponent=aExponent; }
	double getActivationExponent() const { return _activationExponent; }
	void setUseMusclePhysiology(const bool useIt) { _useMusclePhysiology=useIt; }
	bool getUseMusclePhysiology() const { return _useMusclePhysiology; }
	void setConvergenceCriterion(const double aConvergence) { _convergenceCriterion = aConvergence; }
	double getConvergenceCriterion() { return _convergenceCriterion; }
	void setPrintLevel(const int aLevel) { _printLevel = aLevel; }
	int getPrintLevel() { return _printLevel; }
	void setMaxIterations(const int aIterations) { _maxIterations = aIterations; }
	int getMaxIterations() { return _maxIterations; }
	void setOptimizerDX(const double aDX) { _optimizerDX = aDX; }
	double getOptimizerDX() { return _optimizerDX; }
	void constructJointForceColumnLabels();
	Array<int> findJointsToOptimize();
	int findBodyIndexForJoint(std::string &jointName);
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
};	// END of class JointLoadOptimization

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __JointLoadOptimization_h__
