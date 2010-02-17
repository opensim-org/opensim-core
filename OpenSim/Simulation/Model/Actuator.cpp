// Actuator.cpp
// Author: Frank C. Anderson, Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "Actuator.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/DebugUtilities.h>
#include <sstream>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include "SimTKsimbody.h"
#include "OpenSimForceSubsystem.h"
#include "ForceAdapter.h"
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;


//=============================================================================
// STATICS
//=============================================================================
const double Actuator::LARGE = 1.0e8;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct an actuator that has a specified number of controls and state variables.
 *
 * @param aNX Number of controls.
 * @param aNY Number of states.
 */
Actuator::Actuator() :
	_controlSuffixes(""),
	_stateVariableSuffixes(""),
    _subsystemIndex(SimTK::InvalidIndex),
    _numStateVariables(0),
    _controlIndex(-1),
    _isControlled(false),
    _controller(0)
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aActuator Actuator to copy.
 */
Actuator::Actuator(const Actuator &aAct) :
	Force(aAct),
	_controlSuffixes(""),
    _subsystemIndex(SimTK::InvalidIndex),
	_stateVariableSuffixes(""),
    _numStateVariables(0),
    _controlIndex(-1),
    _isControlled(false),
    _controller(0)
{
	setNull();

	// ASSIGN
	*this = aAct;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Actuator::~Actuator()
{
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this Actuator to their null values.
 */
void Actuator::
setNull()
{
	setType("Actuator");
	_model=0;
}

//_____________________________________________________________________________
/**
 */
void Actuator::setNumStateVariables( int aNumStateVariables)
{
	_numStateVariables = aNumStateVariables;
	_stateVariableSuffixes.setSize(aNumStateVariables);
}
//_____________________________________________________________________________
/**
 */
void Actuator::bindStateVariable( int aIndex, const std::string &aSuffix)
{
	_stateVariableSuffixes.set(aIndex, aSuffix);
}
//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aModel model containing this actuator.
 */
void Actuator::setup(Model& aModel)
{
	_model = &aModel;
}

double Actuator::getAppliedForce( const State& s) const {
	return( getForce(s) );
}
double Actuator::getForce( const State& s) const {
    return( Value<double>::downcast(s.getCacheEntry( _subsystemIndex, _forceIndex)).get());
}
void Actuator::setForce( const State& s, double aForce ) const {

    SimTK::Value<double>::downcast(s.updCacheEntry( _subsystemIndex, _forceIndex)).upd() = aForce;
}
double Actuator::getSpeed( const State& s) const {
    return( Value<double>::downcast(s.getCacheEntry( _subsystemIndex, _speedIndex)).get());
}
void Actuator::setSpeed( const State& s, double speed) const {
    SimTK::Value<double>::downcast(s.updCacheEntry( _subsystemIndex, _speedIndex)).upd() = speed;
}
void Actuator::initStateCache(SimTK::State& s, SimTK::SubsystemIndex subsystemIndex, Model& model) {

    _subsystemIndex = subsystemIndex;
    _model = &model;

    _forceIndex = s.allocateCacheEntry( subsystemIndex, SimTK::Stage::Topology, new SimTK::Value<double>() );
    _speedIndex = s.allocateCacheEntry( subsystemIndex, SimTK::Stage::Topology, new SimTK::Value<double>() );

    if( _numStateVariables > 0 ) {
        Vector z(_numStateVariables, 0.0);
        _zIndex = s.allocateZ( subsystemIndex, z);
        _stateVariableDerivIndex = s.allocateCacheEntry( subsystemIndex, SimTK::Stage::Topology, new SimTK::Value<Vector>(z) );
    }

}

//_____________________________________________________________________________
/**
 * Update the geometric representation of the Actuator if any.
 * The resulting geometry is maintained at the VisibleObject layer
 * 
 */
void Actuator::updateGeometry()
{
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
Actuator& Actuator::
operator=(const Actuator &aAct)
{
	// BASE CLASS
	Force::operator=(aAct);

	// MODEL
	_model = &aAct.getModel();

	// NOTE: _x, _y, _yp and subsystem indexes are not copied because they contain pointers that reference within 
	//  this object; it is safe to  copy the suffixes, though possibly unnecessary (since they will be set in 
	//  the object's setNull() before we copy into it).
	_controlSuffixes = aAct._controlSuffixes;
	_stateVariableSuffixes = aAct._stateVariableSuffixes;

	return(*this);
}
// CONTROLS
/**
 * Get the index of a control .
 *
 * @return The index of the control.
 */
int Actuator::getControlIndex() const {
	return _controlIndex;
}
void Actuator::setControlIndex(int index) {
	_controlIndex = index;
}
void Actuator::setController(const Controller* controller ) {
	_controller = controller;
}
const Controller& Actuator::getController() const { 
   return  *_controller;
}
//_____________________________________________________________________________
/**
 * Get an actuator control, by index.
 *
 * @param aIndex the index of the control to get.
 * @return The value of the control.
 */
double Actuator::getControl(const SimTK::State& s ) const
{
      if( _controller ) {
//std::cout << "Actuator::getControl " << getName() << "  index=" << _controlIndex << "   controller: " << _controller->getName() << "  control=" << _controller->computeControl(s, _controlIndex ) << std::endl;
          return( _controller->computeControl(s, _controlIndex ));
	  }
      else{
//std::cout << "Actuator::getControl " << getName() << "no controller" << _controlIndex << "   control= 0.0" << std::endl;
          return(0.0);
      }
}


// STATES

int Actuator::getNumStateVariables() const
{
    return _numStateVariables;
}

//_____________________________________________________________________________
/**
 * Get the name of a state variable, given its index.
 *
 * @param aIndex The index of the state variable to get.
 * @return The name of the state variable.
 */
string Actuator::getStateVariableName(int aIndex) const
{
	if(0<=aIndex && aIndex<_numStateVariables)
		return getName() + "." + _stateVariableSuffixes[aIndex];
	else {
		std::stringstream msg;
		msg << "Actuator::getStateVariableName: ERR- index out of bounds.\nActuator " 
			 << getName() << " of type " << getType() << " has " << getNumStateVariables() << " state variables.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

//_____________________________________________________________________________
/**
 * Get the index of a state, given the name.
 *
 * @param aName The name of the state to get.
 * @return The index of the state.
 */
int Actuator::getStateVariableIndex(const string &aName) const
{
	for(int i=0;i<_stateVariableSuffixes.getSize();i++)
		if(getName()+"."+_stateVariableSuffixes[i]==aName)
			return i;
	return -1;
}

//_____________________________________________________________________________
/**
 * Set an actuator state, specified by index
 *
 * @param aIndex The index of the state to set.
 * @param aValue The value to set the state to.
 */
void Actuator::setStateVariable(SimTK::State& s, int aIndex, double aValue) const {
    Vector& z = s.updZ( _subsystemIndex);
	if(0<=aIndex && aIndex<_numStateVariables) {
		z[_zIndex+aIndex] = aValue;
	} else {
		std::stringstream msg;
		msg << "Actuator::setStateVariable: ERR- index out of bounds.\nActuator " 
			 << getName() << " of type " << getType() << " has " << getNumStateVariables() << " states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

//_____________________________________________________________________________
/**
 * Set all of the states of an actuator.
 *
 * @param aY The array of states to set.
 */

void Actuator::setStateVariables(SimTK::State& s, const double aY[]) const 
{
    Vector& z = s.updZ(_subsystemIndex );
	for(int i=0;i<_numStateVariables;i++) {
		z[_zIndex+i]=aY[i];
	}
}

//_____________________________________________________________________________
/**
 * Get an actuator state, by index.
 *
 * @param aIndex the index of the state to get.
 * @return The value of the state.
 */
double Actuator::getStateVariable(const SimTK::State& s, int aIndex) const
{
    const Vector& z = s.getZ(_subsystemIndex);

	if(0<=aIndex && aIndex<_numStateVariables) {
        return( z[_zIndex+aIndex] );

	} else {
		std::stringstream msg;
		msg << "Actuator::getStateVariable: ERR- index out of bounds.\nActuator " 
		    << getName() << " of type " << getType() << " has " << getNumStateVariables() << " states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

//_____________________________________________________________________________
/**
 * Get all of the states of the actuator.
 *
 * @param rY The array of states is returned here.
 */
void Actuator::getStateVariables(const SimTK::State& s, double rY[]) const
{
    const Vector& z= s.getZ( _subsystemIndex);
	for(int i=0;i<_numStateVariables;i++) {
		 rY[i] = z[_zIndex+i];
	
	}
}
//_____________________________________________________________________________
/**
 * Set the derivative of an actuator state, specified by index
 *
 * @param aIndex The index of the state to set.
 * @param aValue The value to set the state to.
 */
void Actuator::setStateVariableDeriv(const SimTK::State& s, int aIndex, double aValue) const {

    Vector& stateDeriv =  Value<SimTK::Vector>::downcast(s.updCacheEntry( _subsystemIndex, _stateVariableDerivIndex)).upd();
	if(0<=aIndex && aIndex<_numStateVariables) {
		stateDeriv[aIndex] = aValue;
	} else {
		std::stringstream msg;
		msg << "Actuator::setStateVariableDeriv: ERR- index out of bounds.\nActuator " 
			 << getName() << " of type " << getType() << " has " << getNumStateVariables() << " states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

//_____________________________________________________________________________
/**
 * Set the derivatives of all of the states of an actuator.
 *
 * @param aY The array of states to set.
 */
void Actuator::setStateVariableDerivs(const SimTK::State& s, const double aY[]) const 
{
    Vector& stateDeriv =  Value<SimTK::Vector>::downcast(s.updCacheEntry( _subsystemIndex, _stateVariableDerivIndex)).upd();
	for(int i=0;i<_numStateVariables;i++) {
		stateDeriv[+i]=aY[i];
	}
}

//_____________________________________________________________________________
/**
 * Get the derivative of an actuator state, by index.
 *
 * @param aIndex the index of the state to get.
 * @return The value of the state.
 */
double Actuator::getStateVariableDeriv(const SimTK::State& s, int aIndex) const
{
    const Vector& stateDeriv =  Value<SimTK::Vector>::downcast(s.getCacheEntry( _subsystemIndex, _stateVariableDerivIndex)).get();
	if(0<=aIndex && aIndex<_numStateVariables) {
        return( stateDeriv[aIndex] );
	} else {
		std::stringstream msg;
		msg << "Actuator::getStateVariableDeriv: ERR- index out of bounds.\nActuator " 
		    << getName() << " of type " << getType() << " has " << getNumStateVariables() << " states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

//_____________________________________________________________________________
/**
 * Get the derivatives of all of the states of the actuator.
 *
 * @param rY The array of states is returned here.
 */

void Actuator::getStateVariableDerivs(const SimTK::State& s, double rY[]) const
{

    const Vector& stateDeriv =  Value<SimTK::Vector>::downcast(s.getCacheEntry( _subsystemIndex, _stateVariableDerivIndex)).get();
	for(int i=0;i<_numStateVariables;i++) {
		 rY[i] = stateDeriv[i];
	
	}
}

//_____________________________________________________________________________
/**
 * Replace one of the actuator's functions in the property array.
 *
 * @param aOldFunction the function being replaced.
 * @param aNewFunction the new function.
 */
void Actuator::replacePropertyFunction(OpenSim::Function* aOldFunction, OpenSim::Function* aNewFunction)
{
	if (aOldFunction && aNewFunction) {
		PropertySet& propSet = getPropertySet();

		for (int i=0; i <propSet.getSize(); i++) {
			Property* prop = propSet.get(i);
			if (prop->getType() == Property::ObjPtr) {
				if (prop->getValueObjPtr() == aOldFunction) {
					prop->setValue(aNewFunction);
				}
			}
		}
	}
}

//_____________________________________________________________________________
/**
 * getStress needs to be overridden by derived classes to be usable
 */
double Actuator::getStress(const SimTK::State& s ) const
{
	OPENSIM_ERROR_IF_NOT_OVERRIDDEN();
}
//_____________________________________________________________________________
/**
 * getOptimalForce needs to be overridden by derived classes to be usable
 */
double Actuator::getOptimalForce() const
{
	OPENSIM_ERROR_IF_NOT_OVERRIDDEN();
}
