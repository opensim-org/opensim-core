// MarkerPair.cpp
// Author: Peter Loan
/* Copyright (c)  2005, Stanford University and Peter Loan.
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
#include "MarkerPair.h"

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
MarkerPair::MarkerPair() :
	_markerNames(_markerNamesProp.getValueStrArray())
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
MarkerPair::~MarkerPair()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMarkerPair MarkerPair to be copied.
 */
MarkerPair::MarkerPair(const MarkerPair &aMarkerPair) :
   Object(aMarkerPair),
	_markerNames(_markerNamesProp.getValueStrArray())
{
	setNull();
	copyData(aMarkerPair);
}
//_____________________________________________________________________________
/**
 */
MarkerPair::MarkerPair(const std::string &aName1, const std::string &aName2) :
	_markerNames(_markerNamesProp.getValueStrArray())
{
	setNull();
	_markerNames.append(aName1);
	_markerNames.append(aName2);
}


void MarkerPair::copyData(const MarkerPair &aMarkerPair)
{
	_markerNames = aMarkerPair._markerNames;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this MarkerPair to their null values.
 */
void MarkerPair::setNull()
{
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MarkerPair::setupProperties()
{
	_markerNamesProp.setComment("Names of two markers, the distance between which is used to compute a body scale factor.");
	_markerNamesProp.setName("markers");
	_propertySet.append(&_markerNamesProp);
}

MarkerPair& MarkerPair::operator=(const MarkerPair &aMarkerPair)
{
	// BASE CLASS
	Object::operator=(aMarkerPair);

	copyData(aMarkerPair);

	return(*this);
}

void MarkerPair::getMarkerNames(string& aName1, string& aName2) const
{
	aName1 = _markerNames[0];
	aName2 = _markerNames[1];
}
