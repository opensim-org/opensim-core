// VectorFunctionForActuators.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/*  
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "VectorFunctionForActuators.h"



using namespace OpenSim;
using namespace std;


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
VectorFunctionForActuators::~VectorFunctionForActuators()
{
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
VectorFunctionForActuators::
VectorFunctionForActuators(ModelIntegrand *aIntegrand) :
	VectorFunctionUncoupledNxN(aIntegrand->getModel()->getNA()),
	_f(0.0)
{
	setNull();
	_integrand = aIntegrand;
	_integrator = new IntegRKF(aIntegrand,5.0e-6,1.0e-7);
	_integrator->setMaxDT(1.0e-3);
	_f.setSize(getNX());
}
//_____________________________________________________________________________
/**
 * Construct a function from an XML Element. THIS CONSTRUCTOR IS NOT YET WORKING
 * NEED TO ADD HOW TO DEFINE _nX AND _nY FROM aElement
 *
 * @param aElement XML element.
 */
VectorFunctionForActuators::
VectorFunctionForActuators(DOMElement *aElement) :
	VectorFunctionUncoupledNxN(aElement),
	_f(0.0)
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aVectorFunction Function to copy.
 */
VectorFunctionForActuators::
VectorFunctionForActuators(const VectorFunctionForActuators &aVectorFunction) :
	VectorFunctionUncoupledNxN(aVectorFunction),
	_f(0.0)
{
	setNull();

	// ASSIGN
	setEqual(aVectorFunction);
}
//_____________________________________________________________________________
/**
 * Copy this object.
 *
 * @return Pointer to a copy of this object.
 */
Object* VectorFunctionForActuators::
copy() const
{
	VectorFunctionForActuators *func =
		new VectorFunctionForActuators(*this);
	return(func);
}

//_____________________________________________________________________________
/**
 * Copy this object and modify the copy so that it is consistent
 * with a specified XML element node.   NOT FUNCTIONAL
 *
 * The copy is constructed by first using
 * rd1to3VectorGCVSpline::rd1to3VectorGCVSpline(DOMElement*) in order to establish the
 * XML node.  Then, the assignment operator is used to set all
 * data members of the copy to the values of this object.  Finally, the
 * data members of the copy are updated using rd1to3VectorGCVSpline::updateObject().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this object modified by the specified
 * XML element.
 */
Object* VectorFunctionForActuators::
copy(DOMElement *aElement) const
{
	// CONSTRUCT FUNCTION BASED ON XML ELEMENT
	VectorFunctionForActuators *func =
		new VectorFunctionForActuators(aElement);

	// ASSIGN DATA ACCORDING TO THIS ACTUATOR
	*func = *this;

	// UPDATE DATA CCORDING TO THE XML ELEMENT
	func->updateFromXMLNode();

	return(func);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to NULL values.
 */
void VectorFunctionForActuators::
setNull()
{
	setType("VectorFunctionForActuators");
	_ti = 0.0;
	_tf = 0.0;
	_integrand = NULL;
	_integrator = NULL;
}

//_____________________________________________________________________________
/**
 * Set all member variables equal to the members of another object.
 * Note that this method is private.  It is only meant for copying the data
 * members defined in this class.  It does not, for example, make any changes
 * to data members of base classes.
 */
void VectorFunctionForActuators::
setEqual(const VectorFunctionForActuators &aVectorFunction)
{
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
VectorFunctionForActuators& VectorFunctionForActuators::
operator=(const VectorFunctionForActuators &aVectorFunction)
{
	// BASE CLASS
	VectorFunctionUncoupledNxN::operator=(aVectorFunction);

	// DATA
	setEqual(aVectorFunction);

	return(*this);
}


//=============================================================================
// SET & GET
//=============================================================================
//-----------------------------------------------------------------------------
// INTIAL TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the initial time of the simulation.
 *
 * @param aTI Initial time.
 */
void VectorFunctionForActuators::
setInitialTime(double aTI)
{
	_ti = aTI;
}
//_____________________________________________________________________________
/**
 * Get the initial time of the simulation.
 *
 * @return Initial time.
 */
double VectorFunctionForActuators::
getInitialTime() const
{
	return(_ti);
}

//-----------------------------------------------------------------------------
// FINAL TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the final time of the simulation.
 *
 * @param aTF Final time.
 */
void VectorFunctionForActuators::
setFinalTime(double aTF)
{
	_tf = aTF;
}
//_____________________________________________________________________________
/**
 * Get the final time of the simulation.
 *
 * @return Final time.
 */
double VectorFunctionForActuators::
getFinalTime() const
{
	return(_tf);
}

//-----------------------------------------------------------------------------
// TARGET FORCES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the target actuator forces.
 *
 * @param aF Array of target forces.
 */
void VectorFunctionForActuators::
setTargetForces(const double *aF)
{
	int i,N=getNX();
	for(i=0;i<N;i++) _f[i] = aF[i];
}
//_____________________________________________________________________________
/**
 * Get the target actuator forces.
 *
 * @param rF Array of target forces.
 */
void VectorFunctionForActuators::
getTargetForces(double *rF) const
{
	int i,N=getNX();
	for(i=0;i<N;i++) rF[i] = _f[i];
}


//-----------------------------------------------------------------------------
// INTEGRAND
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the integrand.
 *
 * @return Integrand.
 */
ModelIntegrand* VectorFunctionForActuators::
getIntegrand()
{
	return(_integrand);
}



//=============================================================================
// EVALUATE
//=============================================================================
//_____________________________________________________________________________
/**
 * Evaluate the vector function.
 *
 * @param aX Array of controls.
 * @param aF Array of actuator force differences.
 */
void VectorFunctionForActuators::
evaluate(const double *aX,double *rF)
{
	int i;
	int N = getNX();
	Model *model = _integrand->getModel();
	int nyModel = model->getNY();

	// Controls
	ControlSet *controlSet = _integrand->getControlSet();
	controlSet->setControlValues(_tf,aX);

	// States
	Array<double> y(0.0,_integrand->getSize());
	_integrand->getInitialStates(&y[0]);

	// Integration
	_integrator->integrate(_ti,_tf,&y[0],0.000001);

	// Actuator forces
	Array<double> yModel(0.0,nyModel);
	_integrand->convertStatesIntegrandToModel(_tf,&y[0],&yModel[0]);
	model->setStates(&yModel[0]);
	model->computeActuation();

	// Vector function values
	for(i=0;i<N;i++) {
		rF[i] = model->getActuatorForce(i) - _f[i];
	}
}
//_____________________________________________________________________________
/**
 * Evaluate the vector function.
 *
 * @param aX Array of controls.
 * @param aF Array of actuator force differences.
 */
void VectorFunctionForActuators::
evaluate(const Array<double> &aX,Array<double> &rF)
{
	evaluate(&aX[0],&rF[0]);
}
//_____________________________________________________________________________
/**
 * Evaluate the vector function.
 *
 * @param aX Array of controls.
 * @param aF Array of actuator force differences.
 */
void VectorFunctionForActuators::
evaluate(const Array<double> &aX,Array<double> &rF,
			const Array<int> &aDerivWRT)
{
	cout<<"\n\nVectorFunctionForActuators.evaluate:  ";
	cout<<"Stupid Head! What are you trying to do?\n\n";
}

