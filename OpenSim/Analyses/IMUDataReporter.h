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
 * A class for recording the readings off a synthetic IMU placed on a model
 * during a simulation.
 *
 * @author Ayman Habib
 * @version 1.0
 */
class OSIMANALYSES_API IMUDataReporter : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(IMUDataReporter, Analysis);

public:
    OpenSim_DECLARE_PROPERTY(report_orientations, bool,
            "Report orientation of IMU as quaternion, default is true.");
    OpenSim_DECLARE_PROPERTY(report_angular_velocities, bool,
            "Report angular velocity of the IMU, default is true.");
    OpenSim_DECLARE_PROPERTY(report_linear_accelerations, bool,
            "Report linear acceleration of the IMU, default is true.");

    OpenSim_DECLARE_PROPERTY(imu_frames, std::string,
            "What frames to report. Frames, Bodies, Custom are valid. If Custom populate frame_paths accordingly.");
    OpenSim_DECLARE_LIST_PROPERTY(frame_paths, std::string,
            "ComponentPaths for frames to attach Synthetic IMUs to, if IMU_attachments is set to Custom.");

    //=============================================================================
// DATA
//=============================================================================
private:
    std::vector<OpenSim::ComponentPath> _imuComponents;
    /** Output tables. */
    TableReporter_<SimTK::Quaternion> _orientationsReporter;
    TableReporter_<SimTK::Vec3> _angularVelocityReporter;
    TableReporter_<SimTK::Vec3> _linearAccelerationsReporter;

    std::unique_ptr<Model> _modelLocal;
//=============================================================================
// METHODS
//=============================================================================
public:
    IMUDataReporter(Model *aModel=0);
    IMUDataReporter(const IMUDataReporter &aObject);
    virtual ~IMUDataReporter();

    void setNull();
    // Convenience methods to support reporting all Bodies or all Frames or
    // Frames that can be filtered by type.
    template <class T> void reportAll();
    void reportAllBodies() { reportAll<const OpenSim::Body>(); };
    void reportAllFrames() { reportAll<const OpenSim::Frame>(); };

    // In memory access to IMU data as Tables (Orientations)
    const TimeSeriesTable_<SimTK::Quaternion_<double> >&
    getOrientationsTable() const {
        return _orientationsReporter.getTable();
    }
    // In memory access to IMU data as Tables Angular Velocities)
    const TimeSeriesTable_<SimTK::Vec3>& getAngularVelocitiesTable() const {
        return _angularVelocityReporter.getTable();
    }
    // In memory access to IMU data as Tables (Linear Accelerations)
    const TimeSeriesTable_<SimTK::Vec3>& getLinearAccelerationTable() const {
        return _linearAccelerationsReporter.getTable();
    }

public:
    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    IMUDataReporter& operator=(const IMUDataReporter &aRporter);
#endif
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    // ANALYSIS
    //--------------------------------------------------------------------------
    int begin(const SimTK::State& s ) override;
    int step(const SimTK::State& s, int setNumber ) override;
    int end(const SimTK::State& s ) override;
protected:
    virtual int record(const SimTK::State& s );
    //--------------------------------------------------------------------------
    // IO
    //--------------------------------------------------------------------------
public:
    int printResults(const std::string &aBaseName,const std::string &aDir="",
        double aDT=-1.0,const std::string &aExtension=".sto") override;

private:
    void constructProperties() {
        constructProperty_report_orientations(true);
        constructProperty_report_angular_velocities(true);
        constructProperty_report_linear_accelerations(true);
        constructProperty_imu_frames("Bodies");
        constructProperty_frame_paths();
    }
    //=============================================================================
};  // END of class IMUDataReporter

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef _IMUDATA_REPORTER_h_
