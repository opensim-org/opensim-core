#ifndef OPENSIM_IMU_PLACER_H_
#define OPENSIM_IMU_PLACER_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  IMUPlacer.h                             *
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
#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <Simbody.h>
namespace OpenSim {

class Model;
//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing how to calibrate
 * on a model to match placement of Sensors (IMUs)
 *
 *
 * @author Ayman Habib
 */
class OSIMSIMULATION_API IMUPlacer : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(IMUPlacer, Object);

public:
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    OpenSim_DECLARE_PROPERTY(model_file, std::string,
            "Name/path to the xml .osim file used to load a model to be "
            "calibrated.");

    OpenSim_DECLARE_PROPERTY(base_imu_label, std::string,
            "The label of the base IMU in the orientations_file used to account "
            "for the heading difference between the sensor data and the forward "
            "direction of the model. Leave blank if no heading correction is applied."
            "Default to pelvis_imu.");

    OpenSim_DECLARE_PROPERTY(base_heading_axis, std::string,
            "The axis of the base IMU that corresponds to its heading "
            "direction. Options are 'x', '-x', 'y', '-y', 'z' or '-z'."
            "Default to z".);

    OpenSim_DECLARE_PROPERTY(sensor_to_opensim_rotations, SimTK::Vec3,
            "Space fixed Euler angles (XYZ order) from IMU Space to OpenSim"
            "Default (from Z up) to OpenSim (Y up, X forward)");

    OpenSim_DECLARE_PROPERTY(orientation_file_for_calibration, std::string,
            "Name/path to a .sto file of sensor frame orientations as "
            "quaternions to be used for calibration.");

public:
    virtual ~IMUPlacer();
    IMUPlacer();
    IMUPlacer(const std::string& setupFile);
    bool run(bool visualizeResults = false);
    void setModel(Model& aModel) { _model = &aModel; };
    Model& getCalibratedModel() const;

private:
    void constructProperties();
    /** Pointer to the model being _calibrated. */
    SimTK::ReferencePtr<Model> _model;
    /** Flag indicating if Calibration run has been invoked already */
    bool _calibrated;
};  // END of class IMUPlacer
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_IMU_PLACER_H_


