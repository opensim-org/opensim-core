#ifndef OPENSIM_METABOLIC_MUSCLESET_H_
#define OPENSIM_METABOLIC_MUSCLESET_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  MetabolicMuscleSet.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Tim Dorn                                                        *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

// INCLUDE
#include "MetabolicMuscle.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of metabolic muscle properties.
 *
 * @author Tim Dorn
 */

class OSIMSIMULATION_API MetabolicMuscleSet : public Set<MetabolicMuscle>
{
    OpenSim_DECLARE_CONCRETE_OBJECT(MetabolicMuscleSet, Set<MetabolicMuscle>);
//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // Constructor(s) and Setup
    //--------------------------------------------------------------------------
    MetabolicMuscleSet();
    MetabolicMuscleSet(const MetabolicMuscleSet& aMetabolicMuscleSet);



//=============================================================================
};	// END of class MetabolicMuscleSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_METABOLIC_MUSCLESET_H_
