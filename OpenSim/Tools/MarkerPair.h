#ifndef _MarkerPair_h_
#define _MarkerPair_h_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  MarkerPair.h                           *
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Property.h>
//=============================================================================
//=============================================================================
namespace OpenSim {

/**
 * A class for holding the names of a pair of markers (for making
 * measurements on a model).
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMTOOLS_API MarkerPair : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MarkerPair, Object);

//=============================================================================
// DATA
//=============================================================================
private:

protected:
    OpenSim_DECLARE_LIST_PROPERTY_SIZE(markers, std::string, 2,
            "Names of two markers, the distance between which is used to "
            "compute a body scale factor.");

    //=============================================================================
    // METHODS
    //=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    MarkerPair();
    MarkerPair(const std::string &aName1, const std::string &aName2);
    virtual ~MarkerPair();

    void getMarkerNames(std::string& aName1, std::string& aName2) const;
    const std::string& getMarkerName(int i) const {
        if (getProperty_markers().size() < i + 1)
            throw Exception("MarkerPair: ERROR- Pair has incorrect number of Marker names, 2 required.",
                             __FILE__,__LINE__);
        return get_markers(i);
    }
    void setMarkerName(int i, const std::string& aName) {
        // _markerNames.set(i,aName);
    }

protected:

private:
    void constructProperties();
    //=============================================================================
};  // END of class MarkerPair

}; //namespace
//=============================================================================
//=============================================================================

#endif // __MarkerPair_h__


