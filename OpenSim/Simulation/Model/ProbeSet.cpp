// ProbeSet.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Tim Dorn
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2012 Stanford University
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

#include "ProbeSet.h"
#include <OpenSim/Simulation/Model/Probe.h>

using namespace std;
using namespace OpenSim;


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ProbeSet::~ProbeSet(void)
{
}

//_____________________________________________________________________________
/**
 * Default constructor of a ProbeSet.
 */
ProbeSet::ProbeSet()
{
	setNull();
}

ProbeSet::ProbeSet(Model& model) :
	ModelComponentSet<Probe>(model)
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a ProbeSet.
 */
ProbeSet::ProbeSet(const ProbeSet& aAbsProbeSet):
	ModelComponentSet<Probe>(aAbsProbeSet)
{
	setNull();
	*this = aAbsProbeSet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this ProbeSet to their null values.
 */
void ProbeSet::setNull()
{
	
}

/**
 * Post construction initialization.
 */
void ProbeSet::setup(Model& aModel)
{
	// Base class
	Set<Probe>::setup();

	// Do members
	ModelComponentSet<Probe>::setup(aModel);

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
ProbeSet& ProbeSet::operator=(const ProbeSet &aProbeSet)
{
	Set<Probe>::operator=(aProbeSet);

	return (*this);
}
#endif


