#ifndef _SYNTHETIC_IMUDATA_REPORTER_h_
#define _SYNTHETIC_IMUDATA_REPORTER_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  SyntheticIMUDataReporter.h               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/OpenSense/SyntheticIMU.h>
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
class OSIMANALYSES_API SyntheticIMUDataReporter : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(SyntheticIMUDataReporter, Analysis);

public:
    OpenSim_DECLARE_PROPERTY(report_rotations, bool,
            "Report rotations of Synthetic IMUs as quaternions.");
    OpenSim_DECLARE_PROPERTY(report_angular_velocities, bool,
            "Report angular velocities of Synthetic IMUs.");
    OpenSim_DECLARE_PROPERTY(report_linear_accelerations, bool,
            "Report linear accelerations of Synthetic IMUs.");
    //=============================================================================
// DATA
//=============================================================================
private:
    std::vector<OpenSim::ComponentPath> _imuComponents;
    /** Output tables. */
    TableReporter_<SimTK::Vec3> _rotationsReporter;
    TableReporter_<SimTK::Vec3> _angularVelocityReporter;
    TableReporter_<SimTK::Vec3> _linearAccelerationsReporter;

    std::unique_ptr<Model> _modelLocal;
//=============================================================================
// METHODS
//=============================================================================
public:
    SyntheticIMUDataReporter(Model *aModel=0);
    SyntheticIMUDataReporter(const std::string &aFileName);
    SyntheticIMUDataReporter(const SyntheticIMUDataReporter &aObject);
    virtual ~SyntheticIMUDataReporter();

    void setNull();

    template <class T> void reportAll();
    void reportAllBodies() { reportAll<const OpenSim::Body>(); };
    void reportAllFrames() { reportAll<const OpenSim::Frame>(); };

public:
    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    SyntheticIMUDataReporter& operator=(const SyntheticIMUDataReporter &aRporter);
#endif
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    // ANALYSIS
    //--------------------------------------------------------------------------
    int
        begin(const SimTK::State& s ) override;
    int
        step(const SimTK::State& s, int setNumber ) override;
    int
        end(const SimTK::State& s ) override;
protected:
    virtual int
        record(const SimTK::State& s );
    //--------------------------------------------------------------------------
    // IO
    //--------------------------------------------------------------------------
public:
    int
        printResults(const std::string &aBaseName,const std::string &aDir="",
        double aDT=-1.0,const std::string &aExtension=".sto") override;

//=============================================================================
};  // END of class SyntheticIMUDataReporter

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef _SYNTHETIC_IMUDATA_REPORTER_h_
