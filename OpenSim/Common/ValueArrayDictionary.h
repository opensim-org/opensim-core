/* -------------------------------------------------------------------------- *
 *                            OpenSim:  ValueArrayDictionary.h                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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

#ifndef OPENSIM_VALUEARRAYDICTIONARY_H_
#define OPENSIM_VALUEARRAYDICTIONARY_H_

#include "OpenSim/Common/ValueArray.h"

namespace OpenSim {

class ValueArrayDictionary {
public:
    using Dictionary = std::map<std::string, 
                                std::unique_ptr<AbstractValueArray>>;

    const AbstractValueArray& 
    getValueArrayForKey(const std::string& key) const {
        return *_dictionary.at(key);
    }

    void setValueArrayForKey(const std::string& key, 
                             const AbstractValueArray& abstractValueArray) {
        using Value = std::unique_ptr<AbstractValueArray>;

        _dictionary.emplace(key, 
                            Value{abstractValueArray.clone()});
    }

    void removeArrayForKey(const std::string& key) {
        _dictionary.erase(key);
    }

    std::pair<Dictionary::const_iterator, Dictionary::const_iterator>
    getKeyValueIterator() const {
        return std::make_pair(_dictionary.cbegin(), _dictionary.cend());
    }
private:
    Dictionary _dictionary;
};

}

#endif // OPENSIM_VALUEARRAYDICTIONARY_H_
