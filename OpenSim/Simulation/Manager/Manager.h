#ifndef _Manager_h_
#define _Manager_h_
// Manager.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "SimTKsimbody.h"


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
    SimTK::Integrator* _integ;

	/** Initial time of the simulation. */
	double _ti;
	/** Final time of the simulation. */
	double _tf;
	/** First dt in an integration. */
	double _firstDT;
	
	/** Storage for the states. */
	Storage *_stateStore;

   int _steps;
   /** Number of integration step trys. */
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
	virtual ~Manager();
	Manager(Model&,  SimTK::Integrator&);
	/** A Constructor that does not take a model or controllerSet */
	Manager();	

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

    void setPerformAnalyses( bool performAnalyses) { _performAnalyses =  performAnalyses; }
    void setWriteToStorage( bool writeToStorage) { _writeToStorage =  writeToStorage; }

	// Integrator
	SimTK::Integrator& getIntegrator() const;
    void setIntegrator( SimTK::Integrator*);
	// Initial and final times
	void setInitialTime(double aTI);
	double getInitialTime() const;
	void setFinalTime(double aTF);
	double getFinalTime() const;
	void setFirstDT(double aDT);
	double getFirstDT() const;
       // SEPECIFIED TIME STEP
   void setUseSpecifiedDT(bool aTrueFalse);
   bool getUseSpecifiedDT() const;
   // CONSTANT TIME STEP
   void setUseConstantDT(bool aTrueFalse);
   bool getUseConstantDT() const;
   // DT VECTOR
   const Array<double>& getDTArray();
   void setDTArray(int aN,const double aDT[],double aTI=0.0);
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
	void setStateStorage(Storage& aStorage);
	Storage& getStateStorage() const;

   //--------------------------------------------------------------------------
   //  INTERRUPT
   //--------------------------------------------------------------------------
   void halt();
   void clearHalt();
   bool checkHalt();



//=============================================================================
};	// END of class Manager

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __Manager_h__

