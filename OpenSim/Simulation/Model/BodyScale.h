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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyStrArray.h>

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
protected:
    PropertyStrArray _axisNamesProp;
    Array<std::string>& _axisNames;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    BodyScale();
    BodyScale(const BodyScale &aBodyScale);
    virtual ~BodyScale();

    void copyData(const BodyScale &aBodyScale);

#ifndef SWIG
    BodyScale& operator=(const BodyScale &aBodyScale);
    const Array<std::string>& getAxisNames() const { return _axisNames; }
#endif
    Array<std::string>& getAxisNames() { return _axisNames; }

    void setAxisNames(const Array<std::string> &aAxisNames) { 
        _axisNames = aAxisNames;
        _axisNamesProp.setValueIsDefault(false);
    }

protected:

private:
    void setNull();
    void setupProperties();
//=============================================================================
};  // END of class BodyScale
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __BodyScale_h__


