/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Manager.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 * Contributor(s): Ajay Seth, Carmichael Ong, Nicholas Bianco                 *
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

#include <OpenSim/Common/Assertion.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Common/Component.h>

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
Manager::Manager(Model& model) : _recordStatesTrajectory(false),
        _performAnalyses(true), _writeToStorage(true), _model(&model) {
    _sessionName = _model->getName();
    _integ.reset(new SimTK::RungeKuttaMersonIntegrator(
            _model->getMultibodySystem()));
    _timeStepper.reset(new SimTK::TimeStepper(
            _model->getMultibodySystem(), *_integ));
    _statesTraj.reset(new StatesTrajectory());
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

void Manager::setRecordStatesTrajectory(bool recordStatesTrajectory) {
    _recordStatesTrajectory = recordStatesTrajectory;
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
    _integMethod = integMethod;
    const auto& sys = _model->getMultibodySystem();
    switch (integMethod) {
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

        case IntegratorMethod::SemiExplicitEuler2:
            _integ.reset(new SimTK::SemiExplicitEuler2Integrator(sys));
            break;

        case IntegratorMethod::Verlet:
            _integ.reset(new SimTK::VerletIntegrator(sys));
            break;

        case IntegratorMethod::CPodes:
            _integ.reset(new SimTK::CPodesIntegrator(sys));
            break;

        // case IntegratorMethod::SemiExplicitEuler:
        //    _integ.reset(new SimTK::SemiExplicitEulerIntegrator(
        //             sys, _integFixedStepSize));
        //    break;

        default:
            OPENSIM_THROW(Exception, "Integrator method not recognized.");
    }

    _timeStepper.reset(
        new SimTK::TimeStepper(_model->getMultibodySystem(), *_integ));
}

Manager::IntegratorMethod Manager::getIntegratorMethod() const {
    return _integMethod;
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
   _integ->setFixedStepSize(stepSize);
}

void Manager::setIntegratorFinalTime(double finalTime) {
    _integ->setFinalTime(finalTime);
}

void Manager::setIntegratorInternalStepLimit(int nSteps) {
    _integ->setInternalStepLimit(nSteps);
}

void Manager::setIntegratorUseInfinityNorm(bool useInfinityNorm) {
    _integ->setUseInfinityNorm(useInfinityNorm);
}

void Manager::setIntegratorConstraintTolerance(double tol) {
    _integ->setConstraintTolerance(tol);
}

SimTK::Integrator& Manager::getIntegrator() const {
    return *_integ;
}

//=============================================================================
// RESULTS
//=============================================================================
Storage Manager::getStateStorage() const {
    return *_stateStore;
}

TimeSeriesTable Manager::getStatesTable() const {
    return _stateStore->exportToTable();
}

StatesTrajectory Manager::getStatesTrajectory() const {
    return *_statesTraj;
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
            _model->updControllerSet().storeControls(state,
                    (step < 0) ? getStateStorage().getSize() : step);
        }
    }

    // TODO: repeated calls to integrate() will reset the step, but will keep
    // appending to the analyses. Not ideal.
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

    if (_recordStatesTrajectory) {
        // This check is needed to avoid reappending the same state when
        // integrate() is called multiple times (since the initial state of a
        // subsequent call to integrate() is the last state of the previous
        // call). Note that Storage::append() automatically checks for duplicate
        // time values, so a similar check is not needed above.
        if (!(_statesTraj->getSize() > 0 && step == 0)) {
            _statesTraj->append(state);
        }
    }
}

void Manager::initialize(const SimTK::State& s) {
    OPENSIM_THROW_IF(!_model->hasSystem(), Exception,
        "Model has no System. You must call Model::initSystem() before "
        "calling Manager::initialize().");

    if (_recordStatesTrajectory || _writeToStorage || _performAnalyses) {
        _integ->setReturnEveryInternalStep(true);
    }

    // Initialize the time stepper.
    _timeStepper->setReportAllSignificantStates(true);
    _timeStepper->initialize(s);

    // Initialize the states trajectory.
    if (_recordStatesTrajectory) {
        _statesTraj->clear();
    }

    // Initialize the states storage.
    if (_writeToStorage) {
        _stateStore.reset(new Storage(512));
        _stateStore->setName(_sessionName);
        Array<std::string> stateNames = _model->getStateVariableNames();
        Array<std::string> columnLabels;
        columnLabels.setSize(0);
        columnLabels.append("time");
        for (int i = 0; i < stateNames.getSize(); i++) {
            columnLabels.append(stateNames[i]);
        }
        _stateStore->setColumnLabels(columnLabels);

        // Initialize the controls storage.
        if (_model->isControlled()) {
            _model->updControllerSet().constructStorage();
        }
    }
}

const SimTK::State& Manager::integrate(double finalTime) {

    // Initial state.
    const SimTK::State& s = _integ->getState();

    OPENSIM_THROW_IF(SimTK::isNaN(s.getTime()), Exception,
        "The initial state time is NaN. You must call Manager::initialize() "
        "before calling Manager::integrate().");

    OPENSIM_THROW_IF(finalTime <= s.getTime(), Exception,
        "Expected the final time to be greater than initial time, but received "
        "a final time of {:.2f} and an initial time of {:.2f}.",
        finalTime, s.getTime());

    // Integrate.
    log_info("-------");
    log_info("Manager");
    log_info("-------");
    log_info("Starting integration at initial time {:.2f} s...", s.getTime());
    double cpuTime = SimTK::cpuTime();
    double realTime = SimTK::realTime();
    int step = 0;
    record(s, step++);
    double time = s.getTime();
    auto status = SimTK::Integrator::InvalidSuccessfulStepStatus;
    while (time < finalTime) {
        status = _timeStepper->stepTo(finalTime);
        const SimTK::State& s = _integ->getState();

        if (status == SimTK::Integrator::TimeHasAdvanced ||
                status == SimTK::Integrator::ReachedScheduledEvent ||
                status == SimTK::Integrator::ReachedReportTime) {
            if (s.getTime() <  finalTime) {
                record(s, step++);
            } else {
                record(s, -1);
                break;
            }
        }

        if (_integ->isSimulationOver()) {
            auto reason = _integ->getTerminationReason();
            if (reason == SimTK::Integrator::ReachedFinalTime) {
                OPENSIM_ASSERT(status == SimTK::Integrator::EndOfSimulation);
                break;
            } else {
                log_error("Integration failed due to the following reason: {}",
                    _integ->getTerminationReasonString(
                            _integ->getTerminationReason()));
                return s;
            }
        }
        time = s.getTime();
    }
    cpuTime = SimTK::cpuTime() - cpuTime;
    realTime = SimTK::realTime() - realTime;

    SimTK::Real simFinalTime = _timeStepper->getState().getTime();
    log_info("Integration complete at final time {:.2f} s.", simFinalTime);
    log_info(" - CPU time elapsed: {:.6f} s", cpuTime);
    log_info(" - real time elapsed: {:.6f} s", realTime);
    log_info(" - ratio sim-to-real time: {:.3f}", simFinalTime / realTime);
    log_info(" - integrator method: {}", _integ->getMethodName());
    log_info(" - number of realizations: {}", _integ->getNumRealizations());
    log_info(" - number of steps taken / attempted: {} / {}",
            _integ->getNumStepsTaken(), _integ->getNumStepsAttempted());
    log_info(" - number of projections: {}", _integ->getNumProjections());
    log_info(" - number of iterations: {}", _integ->getNumIterations());

    return _timeStepper->getState();
}

const SimTK::State& Manager::getState() const {
    return _timeStepper->getState();
}
