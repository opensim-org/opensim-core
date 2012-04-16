#ifndef __Schutte1993Muscle_Deprecated_h__
#define __Schutte1993Muscle_Deprecated_h__

// Schutte1993Muscle_Deprecated.h
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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


// INCLUDE
#include "osimActuatorsDLL.h"
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle_Deprecated.h>

#ifdef SWIG
	#ifdef OSIMACTUATORS_API
		#undef OSIMACTUATORS_API
		#define OSIMACTUATORS_API
	#endif
#endif

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM muscle.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMACTUATORS_API Schutte1993Muscle_Deprecated 
:   public ActivationFiberLengthMuscle_Deprecated {
OpenSim_DECLARE_CONCRETE_OBJECT(Schutte1993Muscle_Deprecated, 
                                ActivationFiberLengthMuscle_Deprecated);

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Schutte1993Muscle_Deprecated();
	Schutte1993Muscle_Deprecated(const std::string &aName,double aMaxIsometricForce,double aOptimalFiberLength,double aTendonSlackLength,double aPennationAngle);
	Schutte1993Muscle_Deprecated(const Schutte1993Muscle_Deprecated &aMuscle);
	virtual ~Schutte1993Muscle_Deprecated();

#ifndef SWIG
	Schutte1993Muscle_Deprecated& operator=(const Schutte1993Muscle_Deprecated &aMuscle);
#endif
    void copyData(const Schutte1993Muscle_Deprecated &aMuscle);

	//--------------------------------------------------------------------------
	// GET
	//--------------------------------------------------------------------------
	// Properties
	virtual double getTimeScale() const { return getPropertyValue<double>("time_scale"); }
	virtual double getDamping() const { return getPropertyValue<double>("damping"); }
	virtual bool setTimeScale(double aTimeScale);
	virtual bool setActivation1(double aActivation1);
	virtual bool setActivation2(double aActivation2);

	virtual bool setDamping(double aDamping);
	virtual double getDamping() { return getPropertyValue<double>("damping"); }

	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------
	virtual double computeActuation( const SimTK::State& s ) const;
	virtual double computeIsometricForce(SimTK::State& s, double activation) const;

	virtual const Function& getActiveForceLengthCurve() const;
	virtual bool setActiveForceLengthCurve(const Function& aActiveForceLengthCurve);
	virtual const Function& getPassiveForceLengthCurve() const;
	virtual bool setPassiveForceLengthCurve(const Function& aPassiveForceLengthCurve);
	virtual const Function& getTendonForceLengthCurve() const;
	virtual bool setTendonForceLengthCurve(const Function& aTendonForceLengthCurve);

protected:
	// Model Component Interface
	virtual void setup(Model& aModel);

private:
	double calcNonzeroPassiveForce(const SimTK::State& s, double aNormFiberLength, double aNormFiberVelocity) const;
	double calcFiberVelocity(const SimTK::State& s, double aActivation, double aActiveForce, double aVelocityDependentForce) const;
	double calcTendonForce(const SimTK::State& s, double aNormTendonLength) const;

	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class Schutte1993Muscle_Deprecated
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Schutte1993Muscle_Deprecated_h__