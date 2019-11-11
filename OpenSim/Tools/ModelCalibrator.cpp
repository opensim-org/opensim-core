/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ModelCalibrator.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2019 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "ModelCalibrator.h"
#include <OpenSim/Simulation/Model/Model.h>

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ModelCalibrator::ModelCalibrator() 
{
    constructProperties();
}


ModelCalibrator::ModelCalibrator(const std::string& setupFile)
        : Object(setupFile, true) {
    constructProperties();
    updateFromXMLDocument();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
ModelCalibrator::~ModelCalibrator()
{
}

//=============================================================================
// CONSTRUCTION

//_____________________________________________________________________________
/**
 */
void ModelCalibrator::constructProperties() 
{
    constructProperty_base_imu_label("z");
    constructProperty_base_heading_axis("pelvis_imu");
    constructProperty_sensor_to_opensim_rotations(
            SimTK::Vec3(-SimTK_PI / 2, 0, 0));
    constructProperty_calibration_file_name("");
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * This method scales a model based on user-specified XYZ body scale factors
 * and/or a set of marker-to-marker distance measurements.
 *
 * @param aModel the model to scale.
 * @param aSubjectMass the final mass of the model after scaling.
 * @return Whether the scaling process was successful or not.
 */
bool ModelCalibrator::run(bool visualizeResults) const {
    
    return true;
}
