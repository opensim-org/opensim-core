// CoordinateSet.cpp
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

#include "CoordinateSet.h"
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace std;
using namespace OpenSim;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
CoordinateSet::~CoordinateSet(void)
{
}

//_____________________________________________________________________________
/**
 * Default constructor of a CoordinateSet.
 */
CoordinateSet::CoordinateSet() :
	ModelComponentSet<Coordinate>()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a CoordinateSet.
 */
CoordinateSet::CoordinateSet(const CoordinateSet& aCoordinateSet):
	ModelComponentSet<Coordinate>(aCoordinateSet)
{
	setNull();
	*this = aCoordinateSet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this CoordinateSet to their null values.
 */
void CoordinateSet::setNull()
{
	setType("CoordinateSet");
}
	
/**
  * Populate this flat list of Coordinates given a Model that has been setup
  */
void CoordinateSet::populate(Model& model)
{
	_model = &model;
	// Append Coordinate from Joints coodrinate set to the model's set as pointers
	setMemoryOwner(false);
    setSize(0);

	for(int i=0; i< model.getJointSet().getSize(); i++){
		for(int j=0; j< model.getJointSet().get(i).numCoordinates(); j++){
			// Append a pointer (address) otherwise the model will get a copy that will not be updated properly
			append(&(model.getJointSet()[i].getCoordinateSet()[j]));
		}
	}
}

/**
  * Populate this flat list of Coordinates given a Model that has been setup
  */
void CoordinateSet::setup(Model& model)
{
	ModelComponentSet<Coordinate>::setup(model);
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
#ifndef SWIG
CoordinateSet& CoordinateSet::operator=(const CoordinateSet &aCoordinateSet)
{
	Set<Coordinate>::operator=(aCoordinateSet);
	return (*this);
}
#endif
