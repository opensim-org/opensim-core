/* -------------------------------------------------------------------------- *
 *                          OpenSim: MapObject.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2019 Stanford University and the Authors                *
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

#ifndef OPENSIM_MAP_OBJECT_H_
#define OPENSIM_MAP_OBJECT_H_
 // INCLUDES
#include "Object.h"

namespace OpenSim {
/**
* A class for representing an item in a Map
*
* This Object represents a key-value pair to be placed in a Map. 
* The primary functionality is serialization, so no actual
* methods are added, only using the properties interface.
*
* @author Ayman Habib
*/

class OSIMCOMMON_API MapItem : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MapItem, Object);
public:
    OpenSim_DECLARE_PROPERTY(from_name, std::string,
        "Key used to lookup item in the map.");
    OpenSim_DECLARE_PROPERTY(to_value, std::string,
        "Value for corresponding key in the map.");
public:
    MapItem(const std::string& key, const std::string& val);
    MapItem();
    virtual ~MapItem() = default;
private:
    void constructProperties();
};

/**
* A class for representing a Map from a String to another String
* The class is backed by a List property so search/lookup is linear time
* but it allows clients to create arbitrary serializable objects from 
* scripting/Matlab if needed.
*
* The MapObject is serialized as a list to make the matching of keys 
* and values robust. 
*
* @author Ayman Habib
*/
class OSIMCOMMON_API MapObject : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MapObject, Object);
public:
    OpenSim_DECLARE_LIST_PROPERTY(list_MapItems, MapItem,
        "List of MapItem Objects");
    /** Default constructor */
    MapObject();

    /** Constructor from XML file */
    MapObject(const std::string& aFileName); 

    virtual ~MapObject() = default;

    /** Add a Key-value pair to the Map, the pair is packaged as a MapItem
        for cross-language access. If MapItem with same key exists, an exception
        is thrown and the passed in MapItem is not added. */
    void addItem(const MapItem& newItem) {
        if (containsItem(newItem))
            throw OpenSim::Exception("Attempting to add MapItem with existing key:" + newItem.get_from_name());
        updProperty_list_MapItems().appendValue(newItem);
    };

    /** Test whether a MapItem with same key exists in the MapObject */
    bool containsItem(const MapItem& testItem) const {
        return getProperty_list_MapItems().findIndex(testItem)!=-1;
    };

    /** Convenience method to get the number of MapItems in the MapObject */
    int getNumItems() const {
        return getProperty_list_MapItems().size();
    }
};
}
#endif
