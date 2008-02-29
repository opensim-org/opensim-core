// MusclePointSet.cpp
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

#include "MusclePointSet.h"
#include "AbstractMuscle.h"
#include "MuscleViaPoint.h"

using namespace std;
using namespace OpenSim;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
MusclePointSet::~MusclePointSet(void)
{
}

//_____________________________________________________________________________
/**
 * Default constructor of a MusclePointSet.
 */
MusclePointSet::MusclePointSet() :
	Set<MusclePoint>()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a MusclePointSet.
 */
MusclePointSet::MusclePointSet(const MusclePointSet& aSimmMusclePointSet):
	Set<MusclePoint>(aSimmMusclePointSet)
{
	setNull();
	*this = aSimmMusclePointSet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this MusclePointSet to their null values.
 */
void MusclePointSet::setNull()
{
	setType("MusclePointSet");
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
MusclePointSet& MusclePointSet::operator=(const MusclePointSet &aSimmMusclePointSet)
{
	Set<MusclePoint>::operator=(aSimmMusclePointSet);
	return (*this);
}
#endif

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Replace a muscle point in the set with another point. The new one is made a
 * member of all the same groups as the old one, and is inserted in the same
 * place the old one occupied.
 *
 *	@param aOldMusclePoint Muscle point to remove.
 *	@param aNewMusclePoint Muscle point to add.
 */
bool MusclePointSet::replaceMusclePoint(MusclePoint* aOldMusclePoint, MusclePoint* aNewMusclePoint)
{
	if (aOldMusclePoint != NULL && aNewMusclePoint != NULL) {
		AbstractMuscle* muscle = aOldMusclePoint->getMuscle();
		Model* model = muscle->getModel();
		if (muscle != NULL && model != NULL) {
			int count = 0;
	      int index = getIndex(aOldMusclePoint);
			// If you're switching from non-via to via, check to make sure that the muscle
			// will be left with at least 2 non-via points.
			MuscleViaPoint* oldVia = dynamic_cast<MuscleViaPoint*>(aOldMusclePoint);
			MuscleViaPoint* newVia = dynamic_cast<MuscleViaPoint*>(aNewMusclePoint);
			if (oldVia == NULL && newVia != NULL) {
			   for (int i=0; i<getSize(); i++) {
				   if (i != index) {
				      if (dynamic_cast<MuscleViaPoint*>(get(i)) == NULL)
					      count++;
				   }
				}
			} else {
				count = 2;
			}
		   if (count >= 2 && index >= 0) {
			   replace(index, aNewMusclePoint);
			   aNewMusclePoint->setup(model, muscle);
			   muscle->invalidatePath();
				return true;
		   }
		}
	}
	return false;
}
