#ifndef _StaticOptimization_h_
#define _StaticOptimization_h_
// StaticOptimization.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Jeff Reinbolt
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
#include <SimTKcommon.h>


//=============================================================================
//=============================================================================
/**
 */
namespace OpenSim { 

class Model;
class ActuatorSet;

class OSIMANALYSES_API StaticOptimization : public Analysis 
{
	OPENSIM_DECLARE_DERIVED(StaticOptimization, Analysis);
//=============================================================================
// DATA
//=============================================================================
private:

protected:
	/** Use actuator set from model. */
	PropertyBool _useModelActuatorSetProp;
	bool &_useModelActuatorSet;

	PropertyDbl _activationExponentProp;
	double &_activationExponent;

	PropertyBool _useMusclePhysiologyProp;
	bool	&_useMusclePhysiology;

	Storage *_activationStorage;
	Storage *_forceStorage;

	Array<double> _dydt;
	Array<int> _accelerationIndices;

	SimTK::Vector _parameters;

	bool _ownsActuatorSet;
	ActuatorSet *_actuatorSet;

	SimTK::Matrix _performanceMatrix;
	SimTK::Vector _performanceVector;
	SimTK::Matrix _constraintMatrix;
	SimTK::Vector _constraintVector;
	SimTK::Vector _lapackWork;

	double _optimizerDX;
	std::string _optimizerAlgorithm;
	int _printLevel;
	double _convergenceCriterion;
	int _maxIterations;

//=============================================================================
// METHODS
//=============================================================================
public:
	StaticOptimization(Model *aModel=0);
	// Copy constrctor and virtual copy 
	StaticOptimization(const StaticOptimization &aObject);
	virtual Object* copy() const;
	virtual ~StaticOptimization();
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	StaticOptimization& operator=(const StaticOptimization &aStaticOptimization);
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

	bool getUseModelActuatorSet() { return _useModelActuatorSet; }
	void setUseModelActuatorSet(bool aUseModelActuatorSet) { _useModelActuatorSet = aUseModelActuatorSet; }

	virtual void setModel(Model *aModel);
	void setActivationExponent(const double aExponent) { _activationExponent=aExponent; }
	double getActivationExponent() const { return _activationExponent; }
	void setUseMusclePhysiology(const bool useIt) { _useMusclePhysiology=useIt; }
	bool getUseMusclePhysiology() const { return _useMusclePhysiology; }
	//--------------------------------------------------------------------------
	// ANALYSIS
	//--------------------------------------------------------------------------
	virtual int
		begin(int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);
	virtual int
		step(double *aXPrev,double *aYPrev,double *aYPPrev,int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);
	virtual int
		end(int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);
protected:
	virtual int
		record(double aT,double *aX,double *aY,double *aDYDT);

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
public:
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class StaticOptimization

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __StaticOptimization_h__
