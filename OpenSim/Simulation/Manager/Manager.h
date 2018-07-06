#ifndef OPENSIM_MANAGER_H_
#define OPENSIM_MANAGER_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Manager.h                             *
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


// INCLUDES
#include <OpenSim/Common/Array.h>
#include "OpenSim/Common/TimeSeriesTable.h"
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <SimTKcommon/internal/ReferencePtr.h>

namespace SimTK {
class Integrator;
class State;
class System;
class TimeStepper;
}

namespace OpenSim { 

class Model;
class Storage;
class ControllerSet;

//=============================================================================
//=============================================================================
/**
 * A class that manages the execution of a simulation. This class uses a
 * SimTK::Integrator and SimTK::TimeStepper to perform the simulation. By
 * default, a Runge-Kutta-Merson integrator is used, but can be changed by
 * using setIntegrator().
 * 
 * In order to prevent an inconsistency between the Integrator and TimeStepper,
 * we only create a TimeStepper once, specifically when we call
 * initialize(SimTK::State&). To ensure this, the Manager will throw
 * an exception if initialize(SimTK::State&) is called more than once. Note
 * that editing the SimTK::State after calling initialize(SimTK::State&)
 * will not affect the simulation.
 *
 * Note that this interface means that you cannot "reinitialize" a Manager.
 * If you make changes to the SimTK::State, a new Manager must be created
 * before integrating again.
 */
class OSIMSIMULATION_API Manager
{

//=============================================================================
// DATA
//=============================================================================
private:
    /** Simulation session name. */
    std::string _sessionName;
    /** Model for which the simulation is performed. */
    SimTK::ReferencePtr<Model> _model;

    /** Integrator. */
    // This is the actual integrator that is used when integrate() is called.
    // Its memory is managed elsewhere; either by the user or by the
    // _defaultInteg smart pointer.
    SimTK::ReferencePtr<SimTK::Integrator> _integ;

    // The integrator that is used when using the model-only constructor.
    // This is allocated only if necessary.
    std::unique_ptr<SimTK::Integrator> _defaultInteg;

    /** TimeStepper */
    std::unique_ptr<SimTK::TimeStepper> _timeStepper;

    /** Storage for the states. */
    std::unique_ptr<Storage> _stateStore;

    /** Flag for signaling a desired halt. */
    bool _halt;

    /** Flag to indicate whether or not specified integration time steps
    should be used.  The specified integration time steps are held in _tVec.
    If _tVec does not contain time steps appropriate for the integration,
    an exception is thrown. */
    bool _specifiedDT;
    /** Flag to indicate whether or not constant (fixed) integration time
    steps should be used.  The constant integration time step is set using
    setDT(). */
    bool _constantDT;
    /** Constant integration time step. */
    double _dt;
    /** Vector of integration time steps. */
    Array<double> _tArray;
    /** Vector of integration time step deltas. */
    Array<double> _dtArray;

    /** Name to be shown by the UI */
    static std::string _displayName;

    /** flag indicating if manager should call Analyses after each step */
    bool _performAnalyses;

    /** flag indicating if manager should write to storage  each step */
    bool _writeToStorage;

    /** controllerSet used for the integration */
    ControllerSet* _controllerSet;


//=============================================================================
// METHODS
//=============================================================================
public:
    /** This constructor cannot be used in MATLAB/Python, since the
     * SimTK::Integrator%s are not exposed in those languages. */
    Manager(Model& model, SimTK::Integrator& integ);
    /** Constructor that takes a model only and internally uses a
     * SimTK::RungeKuttaMersonIntegrator with default settings (accuracy,
     * constraint tolerance, etc.). MATLAB/Python users must use this
     * constructor. */
    Manager(Model& model);
    /** Convenience constructor for creating and initializing a Manager. */
    Manager(Model& model, const SimTK::State& state);
    /** Convenience constructor for creating and initializing a Manager with
      * a specified integrator. */
    Manager(Model& model, const SimTK::State& state, SimTK::Integrator& integ);
    /** <b>(Deprecated)</b> A Constructor that does not take a model or
     * controllerSet. This constructor also does not set an integrator; you
     * must call setIntegrator() on your own. You should use one of the other
     * constructors. */
    DEPRECATED_14("There will be no replacement for this constructor.")
    Manager();

    // This class would not behave properly if copied (we would need to write a
    // complex custom copy constructor, etc.), so don't allow copies.
    Manager(const Manager&) = delete;
    void operator=(const Manager&) = delete;

private:
    void setNull();
    bool constructStorage();
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
public:
    void setSessionName(const std::string &name);
    void setModel(Model& model);
    const std::string& getSessionName() const;
    const std::string& toString() const;

    void setPerformAnalyses(bool performAnalyses)
    { _performAnalyses =  performAnalyses; }
    void setWriteToStorage(bool writeToStorage)
    { _writeToStorage =  writeToStorage; }

    // Integrator
    SimTK::Integrator& getIntegrator() const;
    /** %Set the integrator. The Manager does *not* take ownership of the
     * passed-in integrator.
     */
    void setIntegrator(SimTK::Integrator&);

    // SPECIFIED TIME STEP
    void setUseSpecifiedDT(bool aTrueFalse);
    bool getUseSpecifiedDT() const;
    // CONSTANT TIME STEP
    void setUseConstantDT(bool aTrueFalse);
    bool getUseConstantDT() const;
    // DT VECTOR
    const Array<double>& getDTArray();
    void setDTArray(const SimTK::Vector_<double>& aDT, double aTI = 0.0);
    double getDTArrayDT(int aStep);
    void printDTArray(const char *aFileName=NULL);
    // TIME VECTOR
    const Array<double>& getTimeArray();
    double getTimeArrayTime(int aStep);
    int getTimeArrayStep(double aTime);
    void printTimeArray(const char *aFileName=NULL);
    void resetTimeAndDTArrays(double aTime);
    double getNextTimeArrayTime(double aTime);



    //--------------------------------------------------------------------------
    // EXECUTION
    //--------------------------------------------------------------------------
    /**
    * Initializes the Manager by creating and initializing the underlying 
    * SimTK::TimeStepper. This must be called before calling 
    * Manager::integrate() Subsequent changes to the State object passed in 
    * here will not affect the simulation. Calling this function multiple 
    * times with the same Manager will trigger an Exception.
    */
    void initialize(const SimTK::State& s);
    
    /**
    * Integrate the equations of motion for the specified model, given the current
    * state (at which the integration will start) and a finalTime. You must call
    * Manager::initialize(SimTK::State&) before calling this function.
    *
    * If you must update states or controls in a loop, you must recreate the 
    * manager within the loop (such discontinuous changes are considered "events"
    * and cannot be handled during integration of the otherwise continuous system).
    * The proper way to handle this situation is to create a SimTK::EventHandler.
    *
    * Example: Integrating from time = 1s to time = 2s
    * @code
    * SimTK::State state = model.initSystem();
    * Manager manager(model);
    * state.setTime(1.0);
    * manager.initialize(state);
    * state = manager.integrate(2.0);
    * @endcode
    *
    * Example: Integrating from time = 1s to time = 2s using the
    *          convenience constructor
    * @code
    * SimTK::State state = model.initSystem();
    * state.setTime(1.0);
    * Manager manager(model, state);
    * state = manager.integrate(2.0);
    * @endcode
    *
    * Example: Integrate from time = 0s to time = 10s, in 2s increments
    * @code
    * dTime = 2.0;
    * finalTime = 10.0;
    * int n = int(round(finalTime/dTime));
    * state.setTime(0.0);
    * manager.initialize(state);
    * for (int i = 1; i <= n; ++i) {
    *     state = manager.integrate(i*dTime);
    * }
    * @endcode
    *
    * Example: Integrate from time = 0s to time = 10s, updating the state
    *          (e.g., the model's first coordinate value) every 2s
    * @code
    * dTime = 2.0;
    * finalTime = 10.0;
    * int n = int(round(finalTime/dTime));
    * state.setTime(0.0);
    * for (int i = 0; i < n; ++i) {
    *     model.getCoordinateSet().get(0).setValue(state, 0.1*i);
    *     Manager manager(model);
    *     state.setTime(i*dTime);
    *     manager.initialize(state);
    *     state = manager.integrate((i+1)*dTime);
    * }
    * @endcode
    *
    */
    const SimTK::State& integrate(double finalTime);

    /** Get the current State from the Integrator associated with this 
    Manager. */
    const SimTK::State& getState() const;
    
    double getFixedStepSize(int tArrayStep) const;

    // STATE STORAGE
    bool hasStateStorage() const;
    /** Set the Storage object to be used for storing states. The Manager takes
    ownership of the passed-in Storage. */
    void setStateStorage(Storage& aStorage);
    Storage& getStateStorage() const;
    TimeSeriesTable getStatesTable() const;

   //--------------------------------------------------------------------------
   //  INTERRUPT
   //--------------------------------------------------------------------------
   void halt();
   void clearHalt();
   bool checkHalt();

private:

    // Handles common tasks of some of the other constructors.
    Manager(Model& model, bool dummyVar);

    // Helper functions during initialization of integration
    void initializeStorageAndAnalyses(const SimTK::State& s);
    void initializeTimeStepper(const SimTK::State& s);

    // Helper to record state and analysis values at integration steps.
    // step = 0 is the beginning, step = -1 used to denote the end/final step
    void record(const SimTK::State& s, const int& step);

//=============================================================================
};  // END of class Manager

}; //namespace
//=============================================================================
//=============================================================================

#endif  // OPENSIM_MANAGER_H_

