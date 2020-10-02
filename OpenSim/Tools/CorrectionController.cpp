/* -------------------------------------------------------------------------- *
 *                     OpenSim:  CorrectionController.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Model.h>
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
    TrackingController(), 
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
    _model = NULL;  
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
    // Copy this class's members.
    _kp = aController._kp;
    _kv = aController._kv;
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
    
    SimTK::Vector actControls(1, 0.0);

    int i = 0;
    auto coordinateActuators = getComponentList<CoordinateActuator>();
    for(auto& act : coordinateActuators) {
        const Coordinate* coord = act.getCoordinate();
        if(coord->isConstrained(s) ) {
            actControls =  0.0;
        } 
        else {
            double qval = coord->getValue(s);
            double uval = coord->getSpeedValue(s);

            // COMPUTE EXCITATIONS
            double oneOverFmax = 1.0 / act.getOptimalForce();
            double pErr = qval - yDesired[2*i];
            double vErr = uval - yDesired[2*i+1];
            double pErrTerm = _kp*oneOverFmax*pErr;
            double vErrTerm = _kv*oneOverFmax*vErr;
            actControls = -vErrTerm - pErrTerm;
        }
        act.addInControls(actControls, controls);
        ++i;
    }
}

// for any post XML deserialization initialization
void CorrectionController::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);

    // create an actuator for each generalized coordinate in the model 
    // add these actuators to the model and set their indexes 
    const CoordinateSet& cs = _model->getCoordinateSet();
    auto actuators = model.updComponentList<CoordinateActuator>();

    for(int i=0; i<cs.getSize(); i++) {
        const Coordinate& coord = cs[i];
        const std::string name = coord.getName() + "_corrector";

        CoordinateActuator* actuator = nullptr;

        for (auto& ca : actuators) {
            if (ca.getName() == name) {
                actuator = &ca;
                break;
            }
        }

        if(!actuator) {
            // create the corrector actuator if it doe not already exist
            actuator = new CoordinateActuator();
            actuator->setCoordinate(&cs.get(i));
            actuator->setName(name);
            // Since CorrectionController is creating these actuators for its
            // own devices, it should take ownership of them, so that when
            // the controller is removed, so are all the actuators it added.
            adoptSubcomponent(actuator);
            setNextSubcomponentInSystem(*actuator);
            
            log_info("CorrectionController::extendConnectToModel(): {} added.",
                name);

            actuator->setOptimalForce(1.0);
        }
        
        // Add to the Controller's list of Actuators (no ownership).
        addActuator(*actuator);
   }

    setNumControls(getActuatorSet().getSize());

    log_info("CorrectionController::extendConnectToModel(): "
             "numActuators = {:d}, kv = {:0.3f}, kp = {:0.3f}",
            getNumControls(), _kv, _kp);
}

// for any initialization requiring a state or the complete system 
void CorrectionController::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
}
