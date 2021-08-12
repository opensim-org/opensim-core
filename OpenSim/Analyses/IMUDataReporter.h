#ifndef _IMUDATA_REPORTER_h_
#define _IMUDATA_REPORTER_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  IMUDataReporter.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2021 Stanford University and the Authors                *
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
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/OpenSense/IMU.h>
#include "osimAnalysesDLL.h"


#ifdef SWIG
    #ifdef OSIMANALYSES_API
        #undef OSIMANALYSES_API
        #define OSIMANALYSES_API
    #endif
#endif
//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * A class for recording the readings off an IMU object placed on a model
 * during a simulation.
 *
 * If using this reporter to compute IMU accelerometer signals based on kinematic
 * information only (i.e., a solution from the InverseKinematicsTool), then
 * set the property `compute_accelerations_without_forces` to true. This property
 * will apply forces to the model corresponding to the kinematics that you
 * provide as input so that the correct accelerations are computed. The input
 * kinematics are splined and then prescribed to the model (via the
 * PositionMotion class), and the applied forces are based on derivatives of
 * these splines; therefore, you should ensure that the input kinematics produce
 * the correct derivatives when splined. It is recommended that you don't not
 * compute any quantities near the beginning or end of the time range in your
 * data, since spline derivatives can be inaccurate in these regions.
 *
 * @author Ayman Habib
 * @version 1.0
 */
class OSIMANALYSES_API IMUDataReporter : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(IMUDataReporter, Analysis);

public:
    OpenSim_DECLARE_PROPERTY(report_orientations, bool,
            "Report orientation of IMU as quaternion, default is true.");
    OpenSim_DECLARE_PROPERTY(report_gyroscope_signals, bool,
            "Report angular velocity of the IMU, default is true.");
    OpenSim_DECLARE_PROPERTY(report_accelerometer_signals, bool,
            "Report the IMU accelerometer signals, default is true.");
    OpenSim_DECLARE_PROPERTY(compute_accelerations_without_forces, bool,
            "Ignore external forces and controls, use kinematics to compute "
            "accelerometer signals, default is false.");
    OpenSim_DECLARE_LIST_PROPERTY(frame_paths, std::string,
            "Additional ComponentPaths to frames in the model to which new IMUs "
            "components are attached. IMUs added based on these paths will be "
            "appended to the list of existing IMUs in the model. "
            "Both the existing IMUs and appended IMUs from these frame paths "
            "are included in the data report.");
    //=============================================================================
// DATA
//=============================================================================
private:
    /** Output tables. */
    SimTK::ReferencePtr<TableReporter_<SimTK::Quaternion>> _orientationsReporter;
    SimTK::ReferencePtr<TableReporter_<SimTK::Vec3>> _angularVelocityReporter;
    SimTK::ReferencePtr<TableReporter_<SimTK::Vec3>> _linearAccelerationsReporter;

    SimTK::ResetOnCopy<std::unique_ptr<Model>> _modelLocal;
    //=============================================================================
// METHODS
//=============================================================================
public:
    IMUDataReporter(Model *aModel = nullptr);
    IMUDataReporter(const IMUDataReporter &aObject);
    virtual ~IMUDataReporter();

    void setNull();

    // In memory access to IMU data as Tables (Orientations)
    const TimeSeriesTable_<SimTK::Quaternion_<double> >&
    getOrientationsTable() const {
        return _orientationsReporter->getTable();
    }
    // In memory access to IMU data as Tables Angular Velocities)
    const TimeSeriesTable_<SimTK::Vec3>& getGyroscopeSignalsTable() const {
        return _angularVelocityReporter->getTable();
    }
    // In memory access to IMU data as Tables (Linear Accelerations)
    const TimeSeriesTable_<SimTK::Vec3>& getAccelerometerSignalsTable() const {
        return _linearAccelerationsReporter->getTable();
    }

public:
    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    IMUDataReporter& operator=(const IMUDataReporter &aReporter);
#endif
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    // ANALYSIS
    //--------------------------------------------------------------------------
    int begin(const SimTK::State& s) override;
    int step(const SimTK::State& s, int setNumber) override;
    int end(const SimTK::State& s) override;
protected:
    virtual int record(const SimTK::State& s);
    //--------------------------------------------------------------------------
    // IO
    //--------------------------------------------------------------------------
public:
    int printResults(const std::string &aBaseName,const std::string &aDir="",
        double aDT=-1.0,const std::string &aExtension=".sto") override;

private:
    void constructProperties() {
        constructProperty_report_orientations(true);
        constructProperty_report_gyroscope_signals(true);
        constructProperty_report_accelerometer_signals(true);
        constructProperty_compute_accelerations_without_forces(false);
        constructProperty_frame_paths();
    }
    //=============================================================================
};  // END of class IMUDataReporter

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef _IMUDATA_REPORTER_h_
