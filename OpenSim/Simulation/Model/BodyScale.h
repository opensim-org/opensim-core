#ifndef __BodyScale_h__
#define __BodyScale_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  BodyScale.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
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

// INCLUDE
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Property.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing how
 * to scale a body segment.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API BodyScale : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(BodyScale, Object);

//=============================================================================
// DATA
//=============================================================================
public:
OpenSim_DECLARE_LIST_PROPERTY_ATMOST(axes, std::string, 3,
        "Axes (X Y Z) along which to scale a body. "
        "For example, 'X Y Z' scales along all three axes, and 'Y' scales "
        "just along the Y axis.")

        //=============================================================================
        // METHODS
        //=============================================================================
        //--------------------------------------------------------------------------
        // CONSTRUCTION
        //--------------------------------------------------------------------------
        public : BodyScale();
    virtual ~BodyScale();

    const Array<std::string> getAxisNames() const {
        return Array<std::string>{get_axes(0), get_axes(1), get_axes(2)};
    }

    void setAxisNames(const Array<std::string>& aAxisNames) {
        set_axes(aAxisNames);
    }

protected:

private:
    void constructProperties();
    //=============================================================================
};  // END of class BodyScale
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __BodyScale_h__


