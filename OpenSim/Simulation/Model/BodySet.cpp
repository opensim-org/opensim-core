// BodySet.cpp
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

#include "BodySet.h"
#include <OpenSim/Common/ScaleSet.h>

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
BodySet::~BodySet(void)
{
}

//_____________________________________________________________________________
/**
 * Default constructor of a BodySet.
 */
BodySet::BodySet() :
	Set<AbstractBody>()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a BodySet.
 */
BodySet::BodySet(const BodySet& aAbsBodySet):
	Set<AbstractBody>(aAbsBodySet)
{
	setNull();
	*this = aAbsBodySet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this BodySet to their null values.
 */
void BodySet::setNull()
{
	setType("BodySet");
}

/**
 * Post construction initialization.
 */
void BodySet::setup(AbstractDynamicsEngine* aAbstractDynamicsEngine)
{
	// Base class
	Set<AbstractBody>::setup();

	// Do members
	for (int i = 0; i < getSize(); i++)
		get(i)->setup(aAbstractDynamicsEngine);

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
BodySet& BodySet::operator=(const BodySet &aAbsBodySet)
{
	Set<AbstractBody>::operator=(aAbsBodySet);
	return (*this);
}
#endif
//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale body set by a set of scale factors
 */
void BodySet::scale(const ScaleSet& aScaleSet, bool aScaleMass)
{
	for(int i=0; i<getSize(); i++) {
		for(int j=0; j<aScaleSet.getSize(); j++) {
			Scale *scale = aScaleSet.get(j);
			if (get(i)->getName() == scale->getSegmentName()) {
				Vec3 scaleFactors(1.0);
				scale->getScaleFactors(scaleFactors);
				get(i)->scale(scaleFactors, aScaleMass);
			}
		}
	}
}
