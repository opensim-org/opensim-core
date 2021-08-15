/* -------------------------------------------------------------------------- *
 *                      OpenSim:  IMUDataReporter.cpp                         *
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
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Analyses/IMUDataReporter.h>
#include <OpenSim/Simulation/OpenSense/IMU.h>
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Simulation/OpenSense/OpenSenseUtilities.h>
#include <OpenSim/Simulation/PositionMotion.h>
using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
IMUDataReporter::~IMUDataReporter()
{ 
}
//_____________________________________________________________________________
/**
 */
IMUDataReporter::IMUDataReporter(Model *aModel) :
    Analysis(aModel) {
        setNull();
        constructProperties();
}
// Copy constructor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
IMUDataReporter::IMUDataReporter(const IMUDataReporter &aIMUDataReporter): 
    Analysis(aIMUDataReporter) {
        setNull();
        // COPY TYPE AND NAME
        *this = aIMUDataReporter;
}

//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
IMUDataReporter& IMUDataReporter::
operator=(const IMUDataReporter& other)
{
    // BASE CLASS
    Analysis::operator=(other);
    copyProperty_report_orientations(other);
    copyProperty_report_gyroscope_signals(other);
    copyProperty_report_accelerometer_signals(other);
    copyProperty_compute_accelerations_without_forces(other);

    copyProperty_frame_paths(other);
    _modelLocal = nullptr;
    return(*this);
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void IMUDataReporter::setNull()
{
    setAuthors("Ayman Habib");

    setName("IMUDataReporter");
    _modelLocal = nullptr;
    // ReferencePtr members are initialized to null by construction
}
//_____________________________________________________________________________
//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the results.
 */
int IMUDataReporter::
record(const SimTK::State& s)
{
    if(_modelLocal == nullptr) return -1;

    // Set model to whatever defaults have been updated to from the last iteration
    SimTK::State& sWorkingCopy = _modelLocal->updWorkingState();
    sWorkingCopy.setTime(s.getTime());

    // update Q's and U's
    sWorkingCopy.setQ(s.getQ());
    sWorkingCopy.setU(s.getU());

    _modelLocal->realizeReport(sWorkingCopy);

    return 0;
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the beginning of an integration 
 *
 * @param s Current state .
 *
 * @return -1 on error, 0 otherwise.
 */
int IMUDataReporter::begin(const SimTK::State& s)
{
    if(!proceed()) return(0);

    OPENSIM_THROW_IF_FRMOBJ(_model == nullptr, Exception, ""
        "IMUDataReporter: model not provided.");

    _modelLocal.reset(_model->clone());

    _orientationsReporter = new TableReporter_<SimTK::Quaternion>();
    _orientationsReporter->setName("OrientationsReporter");
    _angularVelocityReporter = new TableReporter_<SimTK::Vec3>();
    _angularVelocityReporter->setName("AngularVelocityReporter");
    _linearAccelerationsReporter = new TableReporter_<SimTK::Vec3>();
    _linearAccelerationsReporter->setName("LinearAccelerationsReporter");

    if (getProperty_frame_paths().size() > 0) {
        std::vector<std::string> paths_string;
        for (int i = 0; i < getProperty_frame_paths().size(); i++) {
            paths_string.push_back(get_frame_paths(i));
        }
        OpenSenseUtilities::addModelIMUs(*_modelLocal, paths_string);
    }

    _modelLocal->initSystem();
    for (const auto& comp : _modelLocal->getComponentList<IMU>()) {
        if (get_report_orientations())
            _orientationsReporter->addToReport(
                comp.getOutput("orientation_as_quaternion"), comp.getName());
        if (get_report_gyroscope_signals())
            _angularVelocityReporter->addToReport(
                comp.getOutput("gyroscope_signal"), comp.getName());
        if (get_report_accelerometer_signals())
            _linearAccelerationsReporter->addToReport(
                comp.getOutput("accelerometer_signal"), comp.getName());
    }

    _modelLocal->addComponent(_orientationsReporter.get());
    _modelLocal->addComponent(_angularVelocityReporter.get());
    _modelLocal->addComponent(_linearAccelerationsReporter.get());

    if (get_compute_accelerations_without_forces()) {
        OPENSIM_THROW_IF_FRMOBJ(_statesStore == nullptr, Exception,
            "IMUDataReporter: compute_accelerations_without_forces requires "
            "providing a states file as input.");

        // Create splines for coordinates from statesStore and provide to
        // PositionMotion.
        _modelLocal->initSystem();
        auto statesTraj = StatesTrajectory::createFromStatesStorage(
                *_modelLocal, *_statesStore, true, true, true);

        auto posmot = PositionMotion::createFromStatesTrajectory(
                *_modelLocal, statesTraj);
        posmot->setName("position_motion");
        _modelLocal->addComponent(posmot.release());

        auto& state = _modelLocal->initSystem();
        _modelLocal->getComponent<PositionMotion>("position_motion")
                .setEnabled(state, true);
    }

    _modelLocal->initSystem();

    // RECORD
    int status = record(s);
 
    return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * This method should be overridden in derived classes.  It is
 * included here so that the derived class will not have to implement it if
 * it is not necessary.
 *
 * @param s Current state .
 *
 * @return -1 on error, 0 otherwise.
 */
int IMUDataReporter::step(const SimTK::State& s, int stepNumber)
{
    if(!proceed(stepNumber)) return(0);

    record(s);

    return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * @param s Current state 
 *
 * @return -1 on error, 0 otherwise.
 */
int IMUDataReporter::end( const SimTK::State& s)
{
    if(!proceed()) return(0);

    record(s);

    return(0);
}


//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print results.
 * 
 * The file names are constructed as
 * aDir + "/" + aBaseName + "_" + ComponentName + aExtension
 *
 * @param aDir Directory in which the results reside.
 * @param aBaseName Base file name.
 * @param aDT Desired time interval between adjacent storage vectors.  Linear
 * interpolation is used to print the data out at the desired interval.
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int IMUDataReporter::
printResults(const string &aBaseName,const string &aDir,double aDT,
                 const string &aExtension)
{
    {
        IO::CwdChanger cwd = IO::CwdChanger::changeTo(aDir);
        if (get_report_orientations()) {
            auto& rotationsTable = _orientationsReporter->getTable();
            STOFileAdapter_<SimTK::Quaternion>::write(
                    rotationsTable, aBaseName + "_" + "orientations.sto");
        }
        if (get_report_gyroscope_signals()) {
            auto& angVelTable = _angularVelocityReporter->getTable();
            STOFileAdapter_<SimTK::Vec3>::write(
                    angVelTable, aBaseName + "_" + "angular_velocity.sto");
        }
        if (get_report_accelerometer_signals()) {
            auto& linAccTable = _linearAccelerationsReporter->getTable();
            STOFileAdapter_<SimTK::Vec3>::write(
                    linAccTable, aBaseName + "_" + "linear_accelerations.sto");
        }
    }
    return(0);
}
