// SimbodySimmGencoord.cpp
// Authors: Peter Loan
/*
 * Copyright (c)  2008, Stanford University. All rights reserved. 
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
#include <iostream>
#include <string>
#include <math.h>
#include <float.h>
#include <time.h>

#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>

#include "SimbodySimmGencoord.h"


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;

#define MIN(a,b) ((a)<=(b)?(a):(b))

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodySimmGencoord::~SimbodySimmGencoord()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimbodySimmGencoord::SimbodySimmGencoord()
{
}

//_____________________________________________________________________________
/**
 * Constructor from a coordinate.
 *
 * @param aCoordinate Coordinate that this gencoord is based on.
 */
SimbodySimmGencoord::SimbodySimmGencoord(const Coordinate* aCoordinate)
{
   _coordinate = aCoordinate;
}

//_____________________________________________________________________________
/**
 * Write the gencoord to a [SIMM joint] file.
 *
 * @param aStream File to write to.
 */
void SimbodySimmGencoord::write(ofstream& aStream)
{
   aStream << "begingencoord " << _coordinate->getName() << endl;
   double conversion = 1.0;
   if (_coordinate->getMotionType() == Coordinate::Rotational)
      conversion = 180.0 / SimTK::Pi;
   aStream << "range " << _coordinate->getRangeMin() * conversion << " "
      << _coordinate->getRangeMax() * conversion << endl;
   aStream << "default_value " << _coordinate->getDefaultValue() * conversion << endl;
	aStream << "clamped " << (_coordinate->getDefaultClamped() ? "yes" : "no") << endl;
// JACKM NEEDS STATE	aStream << "locked " << (_coordinate->getLocked() ? "yes" : "no") << endl;
   aStream << "endgencoord" << endl << endl;
}
