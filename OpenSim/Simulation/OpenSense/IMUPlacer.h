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
 * A class implementing a set of parameters describing how to place IMUs
 * on a model to match placement of experimental sensors (IMUs)
 *
 * Calibrates a model by registering IMU frames whose orientations in the
 * sensor world frame are specified, assuming the model's default pose is the
 * calibration pose. The resultant model with IMU frames registered is optionally 
 * written to file. Optional properties are available to identify heading correction
 * to line up the base IMU by its label in the orientation_file_for_calibration
 * e.g. 'pelvis imu' and its heading axis as +/- 'x', 'y', or 'z', are used to 
 * align all the IMU data so that base imu's heading (forward) is in the X direction 
 * of OpenSim's ground frame. 
 *
 * @author Ayman Habib, Ajay Seth
 */
class OSIMSIMULATION_API IMUPlacer : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(IMUPlacer, Object);

public:
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    OpenSim_DECLARE_PROPERTY(model_file, std::string,
            "Name/path to the xml .osim file used to load a model to be "
            "calibrated via placement of IMUs.");

    OpenSim_DECLARE_PROPERTY(base_imu_label, std::string,
            "The label of the base IMU in the orientation_file_for_calibration used to account "
            "for the heading difference between the sensor data and the forward "
            "direction of the model. Leave blank if no heading correction is desired.");

    OpenSim_DECLARE_PROPERTY(base_heading_axis, std::string,
            "The axis of the base IMU that corresponds to its heading "
            "direction. Options are 'x', '-x', 'y', '-y', 'z' or '-z'. "
            "Leave blank if no heading correction is desired.");

    OpenSim_DECLARE_PROPERTY(sensor_to_opensim_rotations, SimTK::Vec3,
            "Space fixed Euler angles (XYZ order) from IMU Space to OpenSim. "
            "Default (0, 0, 0).");

    OpenSim_DECLARE_PROPERTY(orientation_file_for_calibration, std::string,
            "Name/path to a .sto file of sensor frame orientations as "
            "quaternions to be used to place IMUs on the model.");

    OpenSim_DECLARE_PROPERTY(output_model_file, std::string,
            "Name of OpenSim model file (.osim) to write when done placing IMUs.");

public:
    virtual ~IMUPlacer();
    IMUPlacer();
    /** Create an IMUPlacer based on a setup file */
    IMUPlacer(const std::string& setupFile);
    /** Run the calibration method to place IMUs on the model,
     Optionally visualize the model post calibration.
     */
    bool run(bool visualizeResults = false) SWIG_DECLARE_EXCEPTION;
    void setModel(Model& aModel) { _model = &aModel; };
    /** Retrieve the calibrated model. This method will throw if called before 
    the run method is invoked.
    */
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


