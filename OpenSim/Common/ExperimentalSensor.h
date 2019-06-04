/* -------------------------------------------------------------------------- *
 *                          OpenSim: ExperimentalSensor.h                     *
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

#ifndef OPENSIM_EXPERIMENTAL_SENSOR_H_
#define OPENSIM_EXPERIMENTAL_SENSOR_H_
 // INCLUDES
#include "Object.h"

namespace OpenSim {
/**
* A class representing the experimental sensor, such as IMU, and its association
* to a model (component) in OpenSim.
*
* @author Ayman Habib
*/

class OSIMCOMMON_API ExperimentalSensor : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(ExperimentalSensor, Object);
public:
    OpenSim_DECLARE_PROPERTY(name_in_model, std::string,
        "The name of the PhysicalFrame representing a sensor (IMU) in Model. " 
        "When loading sensor data, it will be used as a table column label.");
public:
    ExperimentalSensor(const std::string&  sensorName, const std::string& nameInModel) {
        constructProperties();
        upd_name_in_model() = nameInModel;
        setName(sensorName);
    };
    ExperimentalSensor() {
        constructProperties();
    };
    virtual ~ExperimentalSensor() = default;
private:
    void constructProperties() {
        constructProperty_name_in_model("");
    };
};

}
#endif
