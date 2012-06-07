// UnilateralConstraint.cpp
// Author: Ajay Seth
/*
* Copyright (c)  2009, Stanford University. All rights reserved. 
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
#include "UnilateralConstraint.h"
#include "SimbodyEngine.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
//using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
UnilateralConstraint::UnilateralConstraint() 
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
UnilateralConstraint::~UnilateralConstraint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aUnilateralConstraint UnilateralConstraint to be copied.
 */
UnilateralConstraint::UnilateralConstraint(const UnilateralConstraint &aUnilateralConstraint) :
   Constraint(aUnilateralConstraint)
{
	setNull();
	copyData(aUnilateralConstraint);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one UnilateralConstraint to another.
 *
 * @param aUnilateralConstraint UnilateralConstraint to be copied.
 */
void UnilateralConstraint::copyData(const UnilateralConstraint &aUnilateralConstraint)
{
	Constraint::copyData(aUnilateralConstraint);
}

//_____________________________________________________________________________
/**
 * Set the data members of this UnilateralConstraint to their null values.
 */
void UnilateralConstraint::setNull(void)
{
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param Model containing this UnilateralConstraint.
 */
void UnilateralConstraint::connectToModel(Model& aModel)
{
	// Base class
	Super::connectToModel(aModel);
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
UnilateralConstraint& UnilateralConstraint::operator=(const UnilateralConstraint &aConstraint)
{
	// BASE CLASS
	Constraint::operator=(aConstraint);

	copyData(aConstraint);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
