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
#include "osimToolsDLL.h"
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyBool.h>
#include "MarkerPairSet.h"

#ifdef SWIG
    #ifdef OSIMTOOLS_API
        #undef OSIMTOOLS_API
        #define OSIMTOOLS_API
    #endif
#endif

namespace OpenSim {

class BodyScaleSet;
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
protected:
    PropertyObj _markerPairSetProp;
    MarkerPairSet &_markerPairSet;

    PropertyObj _bodyScaleSetProp;
    BodyScaleSet &_bodyScaleSet;

    PropertyBool _applyProp;
    bool &_apply;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    Measurement();
    Measurement(const Measurement &aMeasurement);
    virtual ~Measurement();

#ifndef SWIG
    Measurement& operator=(const Measurement &aMeasurement);
#endif
   void copyData(const Measurement &aMeasurement);

    BodyScaleSet &getBodyScaleSet() { return _bodyScaleSet; }

    MarkerPairSet& getMarkerPairSet() { return _markerPairSet; }
    int getNumMarkerPairs() const { return _markerPairSet.getSize(); }
    const MarkerPair& getMarkerPair(int aIndex) const { return _markerPairSet[aIndex]; }

    bool getApply() const { return _apply; }
    void setApply(bool aApply) { 
        _apply = aApply;
        _applyProp.setValueIsDefault(false);
    }

    void applyScaleFactor(double aFactor, ScaleSet& aScaleSet);

    /* Register types to be used when reading a Measurement object from xml file. */
    static void registerTypes();

private:
    void setNull();
    void setupProperties();
//=============================================================================
};  // END of class Measurement
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Measurement_h__


