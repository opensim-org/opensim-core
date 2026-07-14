#ifndef __Measurement_h__
#define __Measurement_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  Measurement.h                           *
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
#include "OpenSim/Simulation/Model/BodyScaleSet.h"
#include "OpenSim/Tools/MarkerPairSet.h"
#include "osimToolsDLL.h"

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Property.h>

#ifdef SWIG
    #ifdef OSIMTOOLS_API
        #undef OSIMTOOLS_API
        #define OSIMTOOLS_API
    #endif
#endif

namespace OpenSim {

class MarkerPair;
class ScaleSet;

//=============================================================================
//=============================================================================
/**
 * A class implementing a measurement (the distance between one or more pairs
 * of markers, used to scale a model).
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMTOOLS_API Measurement : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(Measurement, Object);

//=============================================================================
// DATA
//=============================================================================
public:
OpenSim_DECLARE_PROPERTY(MarkerPairSet, MarkerPairSet,
        "Set of marker pairs used to determine the scale factors.");
OpenSim_DECLARE_PROPERTY(BodyScaleSet, BodyScaleSet,
        "Set of bodies to be scaled by this measurement.");
OpenSim_DECLARE_PROPERTY(
        apply, bool, "Flag to turn on and off scaling for this measurement.");

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    Measurement();
    virtual ~Measurement();

    const BodyScaleSet& getBodyScaleSet() const { return get_BodyScaleSet(); }
    const MarkerPairSet& getMarkerPairSet() const {
        return get_MarkerPairSet();
    }

    int getNumMarkerPairs() const;
    const MarkerPair& getMarkerPair(int aIndex) const;

    bool getApply() const { return get_apply(); }
    void setApply(bool aApply) { set_apply(aApply); }

    void applyScaleFactor(double aFactor, ScaleSet& aScaleSet) const;

    /* Register types to be used when reading a Measurement object from xml file. */
    static void registerTypes();

private:
    void constructProperties();
    //=============================================================================
};  // END of class Measurement
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Measurement_h__


