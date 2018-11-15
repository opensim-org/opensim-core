#ifndef __SimbodySimmGencoord_h__
#define __SimbodySimmGencoord_h__
/* -------------------------------------------------------------------------- *
 *                          SimbodySimmGencoord.h                             *
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


// INCLUDES
#include <iostream>
#include <string>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Function.h>

namespace OpenSim {

class Coordinate;

//=============================================================================
//=============================================================================
/**
 * A class to hold a gencoord in a SimbodySimmModel.
 *
 * @authors Peter Loan
 * @version 1.0
 */
class SimbodySimmGencoord
{

//=============================================================================
// DATA
//=============================================================================
protected:
    /** Pointer to the coordinate that this gencoord was created from. */
    const Coordinate* _coordinate;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION AND DESTRUCTION
    //--------------------------------------------------------------------------
public:
    virtual ~SimbodySimmGencoord();
    SimbodySimmGencoord();
    SimbodySimmGencoord(const Coordinate* aCoordinate);
   void write(std::ofstream& aStream);

//=============================================================================
};  // END of class SimbodySimmGencoord
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimbodySimmGencoord_h__


