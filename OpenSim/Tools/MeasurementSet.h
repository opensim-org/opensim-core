#ifndef __MeasurementSet_h__
#define __MeasurementSet_h__
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  MeasurementSet.h                         *
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

#include <OpenSim/Common/Set.h>
#include "Measurement.h"

#ifdef SWIG
    #ifdef OSIMTOOLS_API
        #undef OSIMTOOLS_API
        #define OSIMTOOLS_API
    #endif
#endif

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of measurements.
 *
 * @authors Peter Loan
 * @version 1.0
 */

class OSIMTOOLS_API MeasurementSet : public Set<Measurement> {
OpenSim_DECLARE_CONCRETE_OBJECT(MeasurementSet, Set<Measurement>);

private:
    void setNull();
public:
    MeasurementSet();
    MeasurementSet(const MeasurementSet& aSimmMeasurementSet);
    ~MeasurementSet(void);
    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    MeasurementSet& operator=(const MeasurementSet &aSimmMeasurementSet);
#endif
//=============================================================================
};  // END of class MeasurementSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MeasurementSet_h__
