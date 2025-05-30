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
#include <cstdio>
#include "Manager.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/ControllerSet.h>
#include <OpenSim/Common/Array.h>


using namespace OpenSim;
using namespace std;

#define ASSERT(cond) { if (!(cond)) throw(exception()); }

//=============================================================================
// STATICS
//=============================================================================
std::string Manager::_displayName = "Simulator";

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
Manager::Manager(Model& model) : _model(&model), _performAnalyses(true),
       _writeToStorage(true), _controllerSet(&model.updControllerSet()) {

    setNull();
    constructStorage();
    setSessionName(_model->getName());

    _integ.reset(new SimTK::RungeKuttaMersonIntegrator(
            _model->getMultibodySystem()));
}

Manager::Manager(Model& model, const SimTK::State& state) : Manager(model) {
    initialize(state);
}

Manager::Manager() {
    setNull();
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
void Manager::setNull() {
    _sessionName = "";
    _halt = false;
    _specifiedDT = false;
    _constantDT = false;
    _dt = 1.0e-4;
    _performAnalyses=true;
    _writeToStorage=true;
    _tArray.setSize(0);
    _dtArray.setSize(0);
}

bool Manager::constructStorage() {

    Array<string> columnLabels;

    // STATES
    Array<string> stateNames = _model->getStateVariableNames();
    int ny = stateNames.getSize();
    _stateStore.reset(new Storage(512,"states"));
    columnLabels.setSize(0);
    columnLabels.append("time");
    for(int i=0;i<ny;i++) columnLabels.append(stateNames[i]);
    _stateStore->setColumnLabels(columnLabels);

    return(true);
}


//=============================================================================
// GET AND SET
//=============================================================================
void Manager::setModel(Model& model) {
    if(_model != nullptr){
        // May need to issue a warning here that model was already set to avoid 
        // a leak.
    }

    if (_timeStepper) {
        std::string msg = "Cannot set a new Model on this Manager";
        msg += "after Manager::integrate() has been called at least once.";
        OPENSIM_THROW(Exception, msg);
    }

    _model = &model;

    // STORAGE
    constructStorage();

    // SESSION NAME
    setSessionName(_model->getName());
}

void Manager::setSessionName(const string &aSessionName) {
    _sessionName = aSessionName;
    if(_integ.get() == nullptr) return;

    // STORAGE NAMES
    string name;
    if(hasStateStorage()) {
        name = _sessionName + "_states";
        getStateStorage().setName(name);
    }
}

const string& Manager::getSessionName() const {
    return(_sessionName);
}

const std::string& Manager::toString() const {
    return(_displayName);
}

void Manager::setUseSpecifiedDT(bool aTrueFalse) {
    _specifiedDT = aTrueFalse;
    if(_specifiedDT==true) _constantDT = false;
}

bool Manager::getUseSpecifiedDT() const{
    return(_specifiedDT);
}

void Manager::setUseConstantDT(bool aTrueFalse) {
    _constantDT = aTrueFalse;
    if(_constantDT==true) _specifiedDT = false;
}

bool Manager::getUseConstantDT() const {
    return(_constantDT);
}

const OpenSim::Array<double>& Manager::getDTArray() {
    return(_dtArray);
}

void Manager::setDTArray(const SimTK::Vector_<double>& aDT,double aTI) {
    if(aDT.size() == 0)
        return;

    _dtArray.setSize(0);
    _dtArray.ensureCapacity(aDT.size());
    _tArray.setSize(0);
    _tArray.ensureCapacity(aDT.size() + 1);
    int i;
    for(_tArray.append(aTI), i = 0; i < aDT.size(); ++i) {
        _dtArray.append(aDT[i]);
        _tArray.append(_tArray.getLast() + aDT[i]);
    }
}

double Manager::getDTArrayDT(int aStep) {
    if((aStep<0) || (aStep>=_dtArray.getSize())) {
        printf("Manager.getDTArrayDT: ERR- invalid step.\n");
        return(SimTK::NaN);
    }

    return(_dtArray[aStep]);
}

void Manager::printDTArray(const char *aFileName) {
    // OPEN FILE
    FILE *fp;
    if(aFileName==NULL) {
        fp = stdout;
    } else {
        fp = fopen(aFileName,"w");
        if(fp==NULL) {
            printf("Manager.printDTArray: unable to print to file %s.\n",
                aFileName);
            fp = stdout;
        }
    }

    // PRINT
    int i;
    fprintf(fp,"\n\ndt vector =\n");
    for(i=0;i<_dtArray.getSize();i++) {
        fprintf(fp,"%.16lf",_dtArray[i]);
        if(fp!=stdout) fprintf(fp,"\n");
        else fprintf(fp," ");
    }
    fprintf(fp,"\n");

    // CLOSE
    if(fp!=stdout) fclose(fp);
}

const OpenSim::Array<double>& Manager::getTimeArray() {
    return(_tArray);
}

int Manager::getTimeArrayStep(double aTime) {
    int step = _tArray.searchBinary(aTime);
    return(step);
}

double Manager::getNextTimeArrayTime(double aTime) {
    return(getTimeArrayTime( _tArray.searchBinary(aTime)+1));
}

double Manager::getTimeArrayTime(int aStep) {
    if((aStep<0) || (aStep>=_tArray.getSize())) {
        printf("Manager.getTimeArrayTime: ERR- invalid step.\n");
        return(SimTK::NaN);
    }

    return(_tArray[aStep]);
}

void Manager::printTimeArray(const char *aFileName) {
    // OPEN FILE
    FILE *fp;
    if(aFileName==NULL) {
        fp = stdout;
    } else {
        fp = fopen(aFileName,"w");
        if(fp==NULL) {
            printf("Manager.printTimeArray: unable to print to file %s.\n",
                aFileName);
            fp = stdout;
        }
    }

    // PRINT
    int i;
    fprintf(fp,"\n\ntime vector =\n");
    for(i=0;i<_tArray.getSize();i++) {
        fprintf(fp,"%.16lf",_tArray[i]);
        if(fp!=stdout) fprintf(fp,"\n");
        else fprintf(fp," ");
    }
    fprintf(fp,"\n");

    // CLOSE
    if(fp!=stdout) fclose(fp);
}

void Manager::resetTimeAndDTArrays(double aTime) {
    int size = getTimeArrayStep(aTime);
    _tArray.setSize(size+1);
    _dtArray.setSize(size);
}

double Manager::getFixedStepSize(int tArrayStep) const {
    if( _constantDT )
        return( _dt );
    else {
        if( tArrayStep >= _dtArray.getSize() )
             return( _dtArray[ _dtArray.getSize()-1 ] );
        else
            return(_dtArray[tArrayStep]);
    }
}

//=============================================================================
// INTEGRATOR
//=============================================================================
void Manager::setIntegratorMethod(IntegratorMethod integMethod)
{
    if (_timeStepper) {
        std::string msg = "Cannot set a new integrator on this Manager";
        msg += "after Manager::initialize() has been called.";
        OPENSIM_THROW(Exception, msg);
    }

    const auto& sys = _model->getMultibodySystem();
    switch (integMethod) {
        case IntegratorMethod::CPodes:
            std::cout << "Using CPodes integrator." << std::endl;
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

        //case Integrator::SemiExplicitEuler:
        //    _integ.reset(SimTK::SemiExplicitEulerIntegrator(sys, stepSize));
        //    break;

        case IntegratorMethod::SemiExplicitEuler2:
            _integ.reset(new SimTK::SemiExplicitEuler2Integrator(sys));
            break;

        case IntegratorMethod::Verlet:
            _integ.reset(new SimTK::VerletIntegrator(sys));
            break;

        default:
            std::string msg = "Integrator method not recognized.";
            OPENSIM_THROW(Exception, msg);
    }
}

SimTK::Integrator& Manager::getIntegrator() const {
    return *_integ;
}

void Manager::setIntegratorAccuracy(double accuracy) {
    if (!_integ->methodHasErrorControl()) {
        std::string msg = "Integrator method ";
        msg += _integ->getMethodName();
        msg += " does not support error control.";
        OPENSIM_THROW(Exception, msg);
    }

    _integ->setAccuracy(accuracy);
}

void Manager::setIntegratorMinimumStepSize(double hmin) {
    _integ->setMinimumStepSize(hmin);
}

void Manager::setIntegratorMaximumStepSize(double hmax) {
    _integ->setMaximumStepSize(hmax);
}

//void Manager::setIntegratorFixedStepSize(double stepSize) {
//    _integ->setFixedStepSize(stepSize);
//}

void Manager::setIntegratorInternalStepLimit(int nSteps) {
    _integ->setInternalStepLimit(nSteps);
}

//=============================================================================
// STATES STORAGE
//=============================================================================
void Manager::setStateStorage(Storage& aStorage) {
    _stateStore.reset(&aStorage);
}

Storage& Manager::getStateStorage() const {
    if(!_stateStore)
        throw Exception("Manager::getStateStorage(): Storage is not set");
    return *_stateStore;
}

bool Manager::hasStateStorage() const {
    return _stateStore != nullptr;
}

TimeSeriesTable Manager::getStatesTable() const {
    return getStateStorage().exportToTable();
}

//=============================================================================
// EXECUTION
//=============================================================================
void Manager::initialize(const SimTK::State& s) {
    if (!_integ) {
        throw Exception("Manager::initialize(): "
            "Integrator has not been set. Construct the Manager "
            "with an integrator, or call Manager::setIntegrator().");
    }

    _state = s;

    // Here we call the constructStorage because it is possible that
    // the Model's control storage has already been appended in a
    // previous simulation since the Manager mutates the model
    // (Dimitar Stanev; issue:
    // https://github.com/opensim-org/opensim-core/issues/2865).
    if( _writeToStorage && _model->isControlled())
        _controllerSet->constructStorage();
}

SimTK::State Manager::integrate(double finalTime) {
    int step = 1; // for AnalysisSet::step()

    SimTK::TimeStepper ts(_model->getMultibodySystem(), *_integ);
    ts.initialize(_state);

    ts.stepTo(finalTime);
    SimTK::State s = ts.getState();


    // // Set the final time on the integrator so it can signal EndOfSimulation
    // _integ->setFinalTime(finalTime);

    // // CLEAR ANY INTERRUPT
    // // Halts must arrive during an integration.
    // clearHalt();

    // // CHECK SPECIFIED DT STEPPING
    // double initialTime = s.getTime();
    // if (_specifiedDT) {
    //     if (_tArray.getSize() <= 0) {
    //         string msg = "IntegRKF.integrate: ERR- specified dt stepping not";
    //         msg += "possible-- empty time array.";
    //         throw(Exception(msg));
    //     }
    //     double first = _tArray[0];
    //     double last = _tArray.getLast();
    //     if ((getTimeArrayStep(initialTime)<0) || (initialTime<first) || (finalTime>last)) {
    //         string msg = "IntegRKF.integrate: ERR- specified dt stepping not";
    //         msg += "possible-- time array does not cover the requested";
    //         msg += " integration interval.";
    //         throw(Exception(msg));
    //     }
    // }

    // // RECORD FIRST TIME STEP
    // if (!_specifiedDT) {
    //     resetTimeAndDTArrays(initialTime);
    //     if (_tArray.getSize() <= 0) {
    //         _tArray.append(initialTime);
    //     }
    // }
    // bool fixedStep = false;
    // if (_constantDT || _specifiedDT) fixedStep = true;

    // auto status = SimTK::Integrator::InvalidSuccessfulStepStatus;

    // if (!fixedStep) {
    //     _integ->setReturnEveryInternalStep(true);
    // }

    // _model->realizeVelocity(s);
    // initializeStorageAndAnalyses(s);

    // if (fixedStep) {
    //     _model->realizeAcceleration(s);
    //     record(s, step);
    // }

    // double time = initialTime;
    // double stepToTime = finalTime;

    // if (time >= stepToTime) {
    //     // No integration can be performed.
    //     return getState();
    // }

    // // This should use: status != SimTK::Integrator::EndOfSimulation
    // // but if we do that then repeated calls to integrate (and thus stepTo)
    // // fail to continue on integrating. This seems to be a bug in TimeStepper
    // // status. - aseth
    // while (time < finalTime) {
    //     double fixedStepSize;
    //     if (fixedStep) {
    //         fixedStepSize = getNextTimeArrayTime(time) - time;
    //         if (fixedStepSize + time >= finalTime)  fixedStepSize = finalTime - time;
    //         _integ->setFixedStepSize(fixedStepSize);
    //         stepToTime = time + fixedStepSize;
    //     }

    //     status = _timeStepper->stepTo(stepToTime);

    //     if ( (status == SimTK::Integrator::TimeHasAdvanced) ||
    //          (status == SimTK::Integrator::ReachedScheduledEvent) ) {
    //         const SimTK::State& s = _integ->getState();
    //         record(s, step);
    //         step++;
    //     }
    //     // Check if simulation has terminated for some reason
    //     else if (_integ->isSimulationOver() &&
    //                 _integ->getTerminationReason() !=
    //                     SimTK::Integrator::ReachedFinalTime) {
    //         log_error("Integration failed due to the following reason: {}",
    //             _integ->getTerminationReasonString(_integ->getTerminationReason()));
    //         return getState();
    //     }

    //     time = _integ->getState().getTime();
    //     // CHECK FOR INTERRUPT
    //     if (checkHalt()) break;
    // }

    // // CLEAR ANY INTERRUPT
    // clearHalt();

    // record(_integ->getState(), -1);

    return s;
}

const SimTK::State& Manager::getState() const {
    return _timeStepper->getState();
}

void Manager::initializeStorageAndAnalyses(const SimTK::State& s) {
    if( _writeToStorage ) {
        // STORE STARTING CONTROLS
        if (_model->isControlled()){
            _controllerSet->connectToModel(*_model);
        }

        OPENSIM_THROW_IF(!hasStateStorage(), Exception,
            "Manager::initializeStorageAndAnalyses(): "
            "Expected a Storage to write states into, but none provided.");
    }

    record(s, 0);
}

void Manager::record(const SimTK::State& s, const int& step) {
    // ANALYSES
    if (_performAnalyses) {
        AnalysisSet& analysisSet = _model->updAnalysisSet();
        if (step == 0)
            analysisSet.begin(s);
        else if (step < 0)
            analysisSet.end(s);
        else
            analysisSet.step(s, step);
    }
    if (_writeToStorage) {
        SimTK::Vector stateValues = _model->getStateVariableValues(s);
        StateVector vec;
        vec.setStates(s.getTime(), stateValues);
        getStateStorage().append(vec);
        if (_model->isControlled())
            _controllerSet->storeControls(s,
                (step < 0) ? getStateStorage().getSize() : step);
    }
}

//=============================================================================
// INTERRUPT
//=============================================================================
void Manager::halt() {
    _halt = true;
}

void Manager::clearHalt() {
    _halt = false;
}

bool Manager::checkHalt() {
    return _halt;
}
