// CorrectionController.cpp
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
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include "CorrectionController.h"

using namespace std;
using namespace SimTK;
using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
CorrectionController::~CorrectionController()
{
}
//_____________________________________________________________________________
/**
 * Default constructor
 */
CorrectionController::CorrectionController() :
	TrackingController(), 
    _kp(_kpProp.getValueDbl()), 
    _kv(_kvProp.getValueDbl())
{
	// NULL
	setNull();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML Document
 */
CorrectionController::
CorrectionController(const string &aFileName, bool aUpdateFromXMLNode) :
	TrackingController(aFileName,false), 
   _kp(_kpProp.getValueDbl()), 
   _kv(_kvProp.getValueDbl())
{

	setNull();
	if(aUpdateFromXMLNode) updateFromXMLDocument();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aController Controller to copy.
 */
CorrectionController::
CorrectionController(const CorrectionController &aController) :
	TrackingController(aController), 
    _kp(_kpProp.getValueDbl()), 
    _kv(_kvProp.getValueDbl())
{
	setNull();
	copyData(aController);
}
//_____________________________________________________________________________
/**
 * Copy this CorrectionController and return a pointer to the copy.
 * The copy constructor for this class is used.  This method is called
 * when a description of this controller is read in from an XML file.
 *
 * @return Pointer to a copy of this CorrectionController.
 */
Object* CorrectionController::copy() const
{
	CorrectionController *object = new CorrectionController(*this);
	return object;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void CorrectionController::
setNull()
{
    setupProperties();
	setType("CorrectionController");
    _model = NULL;	
	 _desiredStatesStorage = NULL;
}
/**
 ** Assignment operator.
 **
 ** @return Reference to this object.
 **/
CorrectionController& CorrectionController::operator=(const CorrectionController &aController)
{
    // BASE CLASS
      Controller::operator=(aController);

    copyData(aController);


    return(*this);
}
  

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void CorrectionController::
setupProperties()
{
	_kpProp.setComment("Gain for position errors");
	_kpProp.setName("kp");
	_kpProp.setValue(100.0);
	_propertySet.append( &_kpProp );

	_kvProp.setComment("Gain for velocity errors");
	_kvProp.setName("kv");
	_kvProp.setValue(20.0);
	_propertySet.append( &_kvProp );
}
//_____________________________________________________________________________
/**
 * Copy the member variables of the specified controller.
 */
void CorrectionController::
copyData(const CorrectionController &aController)
{
	// Copy parent class's members first.
	Controller::copyData(aController);

	// Copy this class's members.
	_kp = aController._kp;
	_kv = aController._kv;
	_desiredStatesStorage = aController._desiredStatesStorage;
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// KP AND KV
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the gain for position errors.
 *
 * @return Current position error gain value.
 */
double CorrectionController::
getKp() const
{
	return(_kp);
}
//_____________________________________________________________________________
/**
 * Set the gain for position errors.
 *
 * @param aKp Position error gain for controller will be set to aKp.
 */
void CorrectionController::
setKp(double aKp)
{
	_kp = aKp;
}
//_____________________________________________________________________________
/**
 * Get the gain for velocity errors.
 *
 * @return Current velocity error gain value.
 */
double CorrectionController::
getKv() const
{
	return(_kv);
}
//_____________________________________________________________________________
/**
 * Set the gain for velocity errors.
 *
 * @param aKv Velocity error gain for controller will be set to aKv.
 */
void CorrectionController::setKv(double aKv)
{
	_kv = aKv;
}


//=============================================================================
// COMPUTE
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the controls for Actuator's that this Controller is charge of.
 *
 */
void CorrectionController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const
{
	// NUMBER OF MODEL COORDINATES, SPEEDS, AND ACTUATORS
	// (ALL ARE PROBABLY EQUAL)
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();

	double t = s.getTime();
	
	// GET CURRENT DESIRED COORDINATES AND SPEEDS
	// Note: yDesired[0..nq-1] will contain the generalized coordinates
	// and yDesired[nq..nq+nu-1] will contain the generalized speeds.
	Array<double> yDesired(0.0,nq+nu);
	getDesiredStatesStorage().getDataAtTime(t, nq+nu,yDesired);
	
	double newControl = 0.0;


   	for(int i=0; i<_actuatorSet.getSize(); i++){
		CoordinateActuator* act = dynamic_cast<CoordinateActuator*>(&_actuatorSet[i]);
		SimTK_ASSERT( act,  "CorrectionController::computeControls dynamic cast failed");

		Coordinate *aCoord = act->getCoordinate();
		if( aCoord->isConstrained(s) ) {
			newControl =  0.0;
		} 
		else
		{
			int iqx = aCoord->getStateVariableSystemIndex(aCoord->getName());
			int iux = aCoord->getStateVariableSystemIndex(aCoord->getSpeedName());

    		// COMPUTE EXCITATIONS
			double oneOverFmax = 1.0 / act->getOptimalForce();
			double pErr = s.getY()[iqx] - yDesired[iqx];
			double vErr = s.getY()[iux] - yDesired[iux];
			double pErrTerm = _kp*oneOverFmax*pErr;
			double vErrTerm = _kv*oneOverFmax*vErr;
			newControl = -vErrTerm - pErrTerm;
		}

		SimTK::Vector actControls(1, newControl);
		_actuatorSet[i].addInControls(actControls, controls);
	}
}

void CorrectionController::createSystem(SimTK::MultibodySystem& system) const
{ 
}

// for any post XML deserialization intialization
void CorrectionController::setup(Model& model)
{
	TrackingController::setup(model);

	_actuatorSet.setSize(0);
	_actuatorSet.setMemoryOwner(false);
	
	std::cout << std::endl;

	// create an actuator for each generalized coordinate in the model 
	// add these actuators to the model and set their indexes 
	const CoordinateSet& cs = _model->getCoordinateSet();
	for(int i=0; i<cs.getSize(); i++) {
		std::cout << " CorrectionController::setup() " <<  cs.get(i).getName()+"_corrector" << "  added " << std::endl;
		std::string name = cs.get(i).getName()+"_corrector";
		CoordinateActuator *actuator = NULL;
		if(_model->getForceSet().contains(name)){
			actuator = (CoordinateActuator *)&_model->getForceSet().get(name);
		}
		else{
			actuator = new CoordinateActuator();
			actuator->setCoordinate(&cs.get(i));
			actuator->setName(name);
			_model->updForceSet().append(actuator);
		}
			
		actuator->setOptimalForce(1.0);
		
		_actuatorSet.append(actuator);
   }
	_numControls = _actuatorSet.getSize();

	printf(" CorrectionController::setup end  num Actuators= %d kv=%f kp=%f \n",  _model->getForceSet().getSize(), _kv, _kp );
}

// for any intialization requiring a state or the complete system 
void CorrectionController::initState( SimTK::State& s) const
{
}
