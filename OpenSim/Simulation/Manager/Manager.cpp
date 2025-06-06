/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Manager.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

/* Note: This code was originally developed by Realistic Dynamics Inc.
 * Author: Frank C. Anderson
 */
#include "Manager.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/ControllerSet.h>
#include <OpenSim/Common/Array.h>

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
Manager::Manager(Model& model) : _reportStates(true), _performAnalyses(true),
        _writeToStorage(true), _fixedStepSize(SimTK::NaN), _model(&model) {
    _sessionName = _model->getName();
    _integ.reset(new SimTK::RungeKuttaMersonIntegrator(
            _model->getMultibodySystem()));
    _timeStepper.reset(new SimTK::TimeStepper(
            _model->getMultibodySystem(), *_integ));
    _states.reset(new StatesTrajectory());
    _stateStore.reset(new Storage());
}

Manager::Manager(Model& model, const SimTK::State& state) : Manager(model) {
    initialize(state);
}
//=============================================================================
// GET AND SET
//=============================================================================
void Manager::setSessionName(const std::string &aSessionName) {
    _sessionName = aSessionName;
}

const std::string& Manager::getSessionName() const {
    return _sessionName;
}

void Manager::setReportStates(bool reportStates) {
    _reportStates = reportStates;
}

void Manager::setPerformAnalyses(bool performAnalyses) {
    _performAnalyses = performAnalyses;
}

void Manager::setWriteToStorage(bool writeToStorage) {
    _writeToStorage = writeToStorage;
}

//=============================================================================
// INTEGRATOR
//=============================================================================
void Manager::setIntegratorMethod(IntegratorMethod integMethod) {
    _integratorMethod = integMethod;
    const auto& sys = _model->getMultibodySystem();
    switch (integMethod) {
        case IntegratorMethod::CPodes:
            _integ.reset(new SimTK::CPodesIntegrator(sys));
            break;

        case IntegratorMethod::ExplicitEuler:
            _integ.reset(new SimTK::ExplicitEulerIntegrator(sys));
            break;

        case IntegratorMethod::RungeKutta2:
            _integ.reset(new SimTK::RungeKutta2Integrator(sys));
            break;

        case IntegratorMethod::RungeKutta3:
            _integ.reset(new SimTK::RungeKutta3Integrator(sys));
            break;

        case IntegratorMethod::RungeKuttaFeldberg:
            _integ.reset(new SimTK::RungeKuttaFeldbergIntegrator(sys));
            break;

        case IntegratorMethod::RungeKuttaMerson:
            _integ.reset(new SimTK::RungeKuttaMersonIntegrator(sys));
            break;

        case IntegratorMethod::SemiExplicitEuler:
            OPENSIM_THROW_IF(SimTK::isNaN(_fixedStepSize), Exception,
                "The SemiExplicitEuler integrator requires a fixed step size. "
                "Call Manager::setIntegratorFixedStepSize() before calling "
                "Manager::setIntegratorMethod() for this integrator.");

           _integ.reset(new SimTK::SemiExplicitEulerIntegrator(sys, _fixedStepSize));
           break;

        case IntegratorMethod::SemiExplicitEuler2:
            _integ.reset(new SimTK::SemiExplicitEuler2Integrator(sys));
            break;

        case IntegratorMethod::Verlet:
            _integ.reset(new SimTK::VerletIntegrator(sys));
            break;

        default:
            OPENSIM_THROW(Exception, "Integrator method not recognized.");
    }

    _timeStepper.reset(
        new SimTK::TimeStepper(_model->getMultibodySystem(), *_integ));
}

Manager::IntegratorMethod Manager::getIntegratorMethod() const {
    return _integratorMethod;
}

SimTK::Integrator& Manager::getIntegrator() const {
    return *_integ;
}

void Manager::setIntegratorAccuracy(double accuracy) {
    OPENSIM_THROW_IF(!_integ->methodHasErrorControl(), Exception,
        "Integrator method {} does not support error control.", 
        _integ->getMethodName());

    _integ->setAccuracy(accuracy);
}

void Manager::setIntegratorMinimumStepSize(double hmin) {
    _integ->setMinimumStepSize(hmin);
}

void Manager::setIntegratorMaximumStepSize(double hmax) {
    _integ->setMaximumStepSize(hmax);
}

void Manager::setIntegratorFixedStepSize(double stepSize) {
   _fixedStepSize = stepSize;
   _integ->setFixedStepSize(stepSize);
}

void Manager::setIntegratorInternalStepLimit(int nSteps) {
    _integ->setInternalStepLimit(nSteps);
}

void Manager::setIntegratorUseInfinityNorm(bool tf) {
    _integ->setUseInfinityNorm(tf);
}

void Manager::setIntegratorConstraintTolerance(double tol) {
    _integ->setConstraintTolerance(tol);
}

//=============================================================================
// RESULTS
//=============================================================================
Storage Manager::getStateStorage() const {

    // TODO: copy safely.
    return *_stateStore;
}

TimeSeriesTable Manager::getStatesTable() const {

    // TODO: access model safely.
    return _states->exportToTable(*_model);
}

TimeSeriesTable Manager::getControlsTable() const {

    // TODO: bug in ControllerSet: https://github.com/opensim-org/opensim-core/issues/2094
    return _model->getControllerSet().getControlTable();
}

StatesTrajectory Manager::getStatesTrajectory() const {

    // TODO: copy safely.
    return *_states;
}

//=============================================================================
// EXECUTION
//=============================================================================
void Manager::record(const SimTK::State& state, int step) {
    if (_writeToStorage) {
        SimTK::Vector stateValues = _model->getStateVariableValues(state);
        StateVector vec;
        vec.setStates(state.getTime(), stateValues);
        _stateStore->append(vec);
        if (_model->isControlled()) {
            _model->realizeVelocity(state);
            _model->updControllerSet().storeControls(state, 
                    (step < 0) ? getStateStorage().getSize() : step);
        }
    }

    if (_performAnalyses) {
        AnalysisSet& analysisSet = _model->updAnalysisSet();
        if (step == 0) {
            analysisSet.begin(state);
        } else if (step < 0) {
            analysisSet.end(state);
        } else {
            analysisSet.step(state, step);
        }
    }

    if (_reportStates) {
        if (step == 0 && _states->getSize() > 0) { return; }
        _states->append(state);
    }
}

void Manager::initialize(const SimTK::State& s) {
    _timeStepper->initialize(s);
    _timeStepper->setReportAllSignificantStates(true);
    if (_reportStates || _writeToStorage || _performAnalyses) {
        _integ->setReturnEveryInternalStep(true);
    } 
    
    // Initialize the states trajectory.
    // TODO: find a smarter way to handle this.
    _states->clear();
    _states->reserve(1024);

    _stateStore.reset(new Storage(1024, "states"));
    Array<std::string> stateNames = _model->getStateVariableNames();
    Array<std::string> columnLabels;
    columnLabels.setSize(0);
    columnLabels.append("time");
    for (int i = 0; i < stateNames.getSize(); i++) {
        columnLabels.append(stateNames[i]);
    }
    _stateStore->setColumnLabels(columnLabels);

    if (_model->isControlled()) {
        _model->updControllerSet().constructStorage();
    }
}

SimTK::State Manager::integrate(double finalTime) {

    // Initial state.
    const SimTK::State& s = _integ->getState();
    std::cout << "Initial time: " << s.getTime() << std::endl;
    if (s.getTime() >= finalTime) {
        log_warn(
            "Initial time ({}) is greater than or equal to final time ({}). "
            "Returning current state without integration.",
            s.getTime(), finalTime);
        return getState();
    }

    // Integrate.
    int step = 0;
    record(s, step++);
    auto status = SimTK::Integrator::InvalidSuccessfulStepStatus;
    while (status != SimTK::Integrator::SuccessfulStepStatus::ReachedReportTime) { 
        std::cout << "Current step: " << step << ", time: " << _integ->getState().getTime() << std::endl;

        status = _timeStepper->stepTo(finalTime);

        if (status == SimTK::Integrator::TimeHasAdvanced ||
                status == SimTK::Integrator::ReachedScheduledEvent) {
            const SimTK::State& s = _integ->getState();
            record(s, step++);
        }

        if (_integ->isSimulationOver()) {
            auto reason = _integ->getTerminationReason();
            if (reason != SimTK::Integrator::ReachedFinalTime) {
                log_error("Integration failed due to the following reason: {}",
                    _integ->getTerminationReasonString(
                            _integ->getTerminationReason()));
                return _integ->getState();
            } 
        }
    }
    record(_integ->getState(), -1);

    std::printf("\nDone. Used %s with %d function calls.\n",
            _integ->getMethodName(), _integ->getNumRealizations());
    std::printf("  %d steps taken out of %d attempted.\n", 
            _integ->getNumStepsTaken(), _integ->getNumStepsAttempted());

    return _timeStepper->getState();
}

const SimTK::State& Manager::getState() const {
    return _timeStepper->getState();
}

//=============================================================================
// DEPRECATED
//=============================================================================
void Manager::setModel(Model& model) {
    log_error("Manager::setModel() is deprecated and no longer does anything. "
             "Set the model using one of the supported constructor instead.");
}

void Manager::setUseSpecifiedDT(bool aTrueFalse) {
    // noop
}

bool Manager::getUseSpecifiedDT() const{
    return false;
}

void Manager::setUseConstantDT(bool aTrueFalse) {
    // noop
}

bool Manager::getUseConstantDT() const {
    return false;
}

OpenSim::Array<double> Manager::getDTArray() {
    return OpenSim::Array<double>();
}

void Manager::setDTArray(const SimTK::Vector_<double>& aDT,double aTI) {
    // noop
}

double Manager::getDTArrayDT(int aStep) {
    return SimTK::NaN;
}

void Manager::printDTArray(const char *aFileName) {
    // noop
}

OpenSim::Array<double> Manager::getTimeArray() {
    return OpenSim::Array<double>();
}

int Manager::getTimeArrayStep(double aTime) {
    return -1;
}

double Manager::getNextTimeArrayTime(double aTime) {
    return SimTK::NaN;
}

double Manager::getTimeArrayTime(int aStep) {
    return SimTK::NaN;
}

void Manager::printTimeArray(const char *aFileName) {
    // noop
}

void Manager::resetTimeAndDTArrays(double aTime) {
    // noop
}

double Manager::getFixedStepSize(int tArrayStep) const {
    return SimTK::NaN;
}

void Manager::halt() {
    // noop
}

void Manager::clearHalt() {
    // noop
}

bool Manager::checkHalt() {
    return false;
}

std::string Manager::toString() const {
    return "";
}

void Manager::setStateStorage(Storage& aStorage) {
    _stateStore.reset(&aStorage);
}

bool Manager::hasStateStorage() const {
    return _stateStore != nullptr;
}
