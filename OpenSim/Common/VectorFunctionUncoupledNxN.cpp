// VectorFunctionUncoupledNxN.cpp
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
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "VectorFunctionUncoupledNxN.h"
#include "PropertyDbl.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
VectorFunctionUncoupledNxN::~VectorFunctionUncoupledNxN()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
VectorFunctionUncoupledNxN::
VectorFunctionUncoupledNxN() :
	VectorFunction(0,0)
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
VectorFunctionUncoupledNxN::
VectorFunctionUncoupledNxN(int aN) :
	VectorFunction(aN,aN)
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aVectorFunction Function to copy.
 */
VectorFunctionUncoupledNxN::
VectorFunctionUncoupledNxN(const VectorFunctionUncoupledNxN &aVectorFunction) :
	VectorFunction(aVectorFunction)
{
	setNull();

	// ASSIGN
	setEqual(aVectorFunction);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to NULL values.
 */
void VectorFunctionUncoupledNxN::
setNull()
{
	setType("VectorFunctionUncoupledNxN");
}

//_____________________________________________________________________________
/**
 * Set all member variables equal to the members of another object.
 * Note that this method is private.  It is only meant for copying the data
 * members defined in this class.  It does not, for example, make any changes
 * to data members of base classes.
 */
void VectorFunctionUncoupledNxN::
setEqual(const VectorFunctionUncoupledNxN &aVectorFunction)
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
VectorFunctionUncoupledNxN& VectorFunctionUncoupledNxN::
operator=(const VectorFunctionUncoupledNxN &aVectorFunction)
{
	// BASE CLASS
	VectorFunction::operator=(aVectorFunction);

	// DATA
	setEqual(aVectorFunction);

	return(*this);
}


//=============================================================================
// SET AND GET
//=============================================================================

