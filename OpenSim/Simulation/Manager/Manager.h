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
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
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
}

namespace OpenSim { 

class Model;
class Storage;
class ControllerSet;

//=============================================================================
//=============================================================================
/**
 * A class that manages the execution of a simulation.
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
    Model *_model;

    /** Integrator. */
    // This is the actual integrator that is used when integrate() is called.
    // Its memory is managed elsewehere; either by the user or by the
    // _defaultInteg smart pointer.
    SimTK::ReferencePtr<SimTK::Integrator> _integ;

    // The integrator that is used when using the model-only constructor.
    // This is allocated only if necessary.
    std::unique_ptr<SimTK::Integrator> _defaultInteg;

    /** Initial time of the simulation. */
    double _ti;
    /** Final time of the simulation. */
    double _tf;
    /** First dt in an integration. */
    double _firstDT;
    
    /** Storage for the states. */
    std::unique_ptr<Storage> _stateStore;

   int _steps;
   /** Number of integration step tries. */
   int _trys;
   /** Maximum number of steps in an integration. */
   int _maxSteps;
   /** Flag for signaling a desired halt. */
   bool _halt;
   /** Minimum step size. */
   double _dtMin;
   /** Maximum step size. */
   double _dtMax;
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

    /** system of equations to be integrated */
    const SimTK::System* _system;


//=============================================================================
// METHODS
//=============================================================================
public:
    /** This constructor cannot be used in MATLAB/Python, since the
     * SimTK::Integrator%s are not exposed in those languages. */
    Manager(Model&, SimTK::Integrator&);
    /** Constructor that takes a model only and internally uses a
     * SimTK::RungeKuttaMersonIntegrator with default settings (accuracy,
     * constraint tolerance, etc.). MATLAB/Python users must use this
     * constructor. */
    Manager(Model& aModel);
    /** <b>(Deprecated)</b> A Constructor that does not take a model or
     * controllerSet. This constructor also does not set an integrator; you
     * must call setIntegrator() on your own. You should use one of the other
     * two constructors. */
    DEPRECATED_14("There will be no replacement for this constructor.")
    Manager();  

    // This class would not behave properly if copied (we would need to write a
    // complex custom copy constructor, etc.), so don't allow copies.
    Manager(const Manager&) = delete;
    void operator=(const Manager&) = delete;

private:
    void setNull();
    bool constructStates();
    bool constructStorage();
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
public:
    void setSessionName(const std::string &name);
    void setModel(Model& aModel);
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

    // Initial and final times
    void setInitialTime(double aTI);
    double getInitialTime() const;
    void setFinalTime(double aTF);
    double getFinalTime() const;
    void setFirstDT(double aDT);
    double getFirstDT() const;
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


    // SYSTEM
    // only called when need to integrate a different set of equations 
    // then what is defined by the model 
    void setSystem(SimTK::System* system) { _system = system; }

    //--------------------------------------------------------------------------
    // EXECUTION
    //--------------------------------------------------------------------------
    bool integrate( SimTK::State& s, double dtFirst=1.0e-6 );
    bool doIntegration( SimTK::State& s, int step, double dtFirst );
    void initialize(SimTK::State& s, double dt);
    void finalize( SimTK::State& s);
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
    Manager(Model& aModel, bool dummyVar);

//=============================================================================
};  // END of class Manager

}; //namespace
//=============================================================================
//=============================================================================

#endif  // OPENSIM_MANAGER_H_

