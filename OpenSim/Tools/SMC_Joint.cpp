// SMC_Joint.cpp
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
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include "SMC_Joint.h"

using namespace std;
using namespace OpenSim;

//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SMC_Joint::~SMC_Joint()
{
}

//_____________________________________________________________________________
/**
 * Construct a task for a specified generalized coordinate.
 *
 * @param aQID ID of the generalized coordinate to be tracked.
 * @todo Insted of an integer id, the name of the coordinate
 * should be used.
 */
SMC_Joint::SMC_Joint(const string &aCoordinateName) :
	CMC_Joint(aCoordinateName),
	_s(_propS.getValueDbl())
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aTask Joint task to be copied.
 */
SMC_Joint::SMC_Joint(const SMC_Joint &aTask) :
	CMC_Joint(aTask),
	_s(_propS.getValueDbl())
{
	setNull();
	copyData(aTask);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void SMC_Joint::
setNull()
{
	setType("SMC_Joint");
	setupProperties();
	_s = 100.0;
}
//_____________________________________________________________________________
/**
 * Set up serialized member variables.
 */
void SMC_Joint::
setupProperties()
{
	_propS.setComment("Parameter for specifying the boundary"
		"of the surface error. The default for this parameter is 100.0."
		" Generally, this parameter can have a value in the range of 1.0 to 1000.0.");
	_propS.setName("surface_error_boundary");
	_propertySet.append(&_propS);
}


//_____________________________________________________________________________
/**
 * Copy only the member data of specified object.
 */
void SMC_Joint::
copyData(const SMC_Joint &aTask)
{
	_s = aTask._s;
}

//_____________________________________________________________________________
/**
 * Copy this track object and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this track object.
 */
Object* SMC_Joint::
copy() const
{
	SMC_Joint *object = new SMC_Joint(*this);
	return(object);
}

//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @param aTask Object to be copied.
 * @return  Reference to the altered object.
 */
SMC_Joint& SMC_Joint::
operator=(const SMC_Joint &aTask)
{
	// BASE CLASS
	CMC_Joint::operator =(aTask);

	// DATA
	copyData(aTask);

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the desired accelerations.
 * This method assumes that the states have been set for the model.
 *
 * @param aT Time at which the desired accelerations are to be computed in
 * real time units.
 * @see Model::set()
 * @see Model::setStates()
 */
void SMC_Joint::
computeDesiredAccelerations( const SimTK::State& state, double aT)
{
	_aDes = SimTK::NaN;

	// CHECK
	if(_model==NULL) return;
	if(_pTrk[0]==NULL) return;

	// COMPUTE ERRORS
	computeErrors(state, aT);

	// Term 1: Experimental Acceleration
	double a;
	if(_aTrk[0]==NULL) {
		std::vector<int> derivComponents(2);
		derivComponents[0]=0;
		derivComponents[1]=0;
		a = (_ka)[0]*_pTrk[0]->calcDerivative(derivComponents,SimTK::Vector(1,aT));
	} else {
		a = (_ka)[0]*_aTrk[0]->calcValue(SimTK::Vector(1,aT));
	}

	// Surface Error
	double s = -_vErr[0] -(_kv)[0]*_pErr[0];

	// Term 2: Velocity
	double v = (_kv)[0]*_vErr[0];

	// Term 3: Robust Term
	double r = (_kp)[0] * tanh(s/_s);

	// DESIRED ACCELERATION
	_aDes[0] = a + v - r;

	// PRINT
	//printf("SMC_Joint.computeDesiredAcceleration:\n");
	//printf("%s:  t=%lf aDes=%lf a=%lf vErr=%lf pErr=%lf\n",getName(),t,_aDes[0],
	//	_pTrk[0]->evaluate(2,t),_vErr[0],_pErr[0]);
}
