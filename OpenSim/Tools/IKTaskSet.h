#ifndef __IKTaskSet_h__
#define __IKTaskSet_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  IKTaskSet.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Eran Guendelman                                                 *
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

#include "osimToolsDLL.h"
#include <OpenSim/Common/Set.h>
#include "IKTask.h"
#include "IKMarkerTask.h"
#include <OpenSim/Simulation/MarkersReference.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * @authors Eran Guendelman
 * @version 1.0
 * - Added constructor from a file for use in GUI. -Ayman 02/20/07
 */

class OSIMTOOLS_API IKTaskSet : public Set<IKTask> {
OpenSim_DECLARE_CONCRETE_OBJECT(IKTaskSet, Set<IKTask>);

public:
    IKTaskSet() {}
    IKTaskSet(const IKTaskSet &aIKTaskSet) : Set<IKTask>(aIKTaskSet) { }
    IKTaskSet(const std::string &aFileName) : Set<IKTask>(aFileName) { }
    void createMarkerWeightSet(Set<MarkerWeight>& aWeights){
        for(int i=0; i< getSize(); i++){
            if(IKMarkerTask *nextTask = dynamic_cast<IKMarkerTask *>(&get(i))){
                if(nextTask->getApply()){
                    aWeights.cloneAndAppend(*(new MarkerWeight(nextTask->getName(), nextTask->getWeight())));
                }
            }
        }
    };
#ifndef SWIG
    IKTaskSet& operator=(const IKTaskSet &aIKTaskSet) { Set<IKTask>::operator=(aIKTaskSet); return *this; }
#endif
//=============================================================================
};  // END of class IKTaskSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __IKTaskSet_h__
