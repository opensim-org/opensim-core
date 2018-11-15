/* -------------------------------------------------------------------------- *
 *                          SimbodySimmGencoord.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, P2C HD065690, U54 EB020405)   *
 * and by DARPA through the Warrior Web program.                              *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
#include <fstream>
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
// JACKM NEEDS STATE    aStream << "locked " << (_coordinate->getLocked() ? "yes" : "no") << endl;
   aStream << "endgencoord" << endl << endl;
}
