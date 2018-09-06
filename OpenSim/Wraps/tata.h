#ifndef OPENSIM_TATA_H_
#define OPENSIM_TATA_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  tata.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matthew Millard, Ajay Seth, Peter Loan                          *
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
#include <OpenSim/Common/Component.h>
#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>
#include <OpenSim/Actuators/MuscleFirstOrderActivationDynamicModel.h>
#include <OpenSim/Actuators/MuscleFixedWidthPennationModel.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {

//==============================================================================
//                               tata
//==============================================================================


class OSIMACTUATORS_API tata : public WrapObject {
OpenSim_DECLARE_CONCRETE_OBJECT(tata, WrapObject);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(coucou, double, "coucouter");

    tata();

    virtual const char* getWrapTypeName() const override;
    virtual int wrapLine(const SimTK::State& state,
                          SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
                          const PathWrap& aPathWrap,
                         WrapResult& aWrapResult, bool& aFlag) const override;

private:
    void setNull();
    void constructProperties();
};
} // end of namespace OpenSim

#endif // OPENSIM_TATA_H_
