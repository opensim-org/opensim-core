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
#include <OpenSim/Simulation/Model/ModelIntegrand.h>
#include <OpenSim/Simulation/Integrator/IntegRKF.h>


//=============================================================================
//=============================================================================
/**
 * A class that manages the execution of a simulation.
 */
namespace OpenSim { 

class Model;

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
	/** Array of integrated states. */
	Array<double> _y;
	/** Number of model pseudostates. */
	int _nyp;
	/** Array of model pseudostates. */
	Array<double> _yp;
	/** Integrand for the model. */
	ModelIntegrand *_integrand;
	/** Integrator. */
	IntegRKF *_integ;
	/** Initial time of the simulation. */
	double _ti;
	/** Final time of the simulation. */
	double _tf;
	/** First dt in an integration. */
	double _firstDT;
	/** Name to be shown by the UI */
	static std::string _displayName;


//=============================================================================
// METHODS
//=============================================================================
public:
	virtual ~Manager();
	Manager(ModelIntegrand *aIntegrand);
	/** A Constructor that does not take a model or controlset */
	Manager();	

private:
	void setNull();
	bool constructStates();
	bool constructIntegrator();
	bool constructStorage();

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	void setSessionName(const std::string &name);
	const std::string& getSessionName() const;
	const std::string& toString() const;
	// Integrand
	void setIntegrand(ModelIntegrand *aIntegrand);
	ModelIntegrand* getIntegrand() const;
	// Integrator
	IntegRKF* getIntegrator() const;
	// Initial and final times
	void setInitialTime(double aTI);
	double getInitialTime() const;
	void setFinalTime(double aTF);
	double getFinalTime() const;
	void setFirstDT(double aDT);
	double getFirstDT() const;

	//--------------------------------------------------------------------------
	// EXECUTION
	//--------------------------------------------------------------------------
	bool initializeStates();
	bool initializeStates(double *aY,double *aYP=NULL);
	bool integrate();
	bool integrate(int startIndex);
	bool integrate(double startTime);


//=============================================================================
};	// END of class Manager

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __Manager_h__

