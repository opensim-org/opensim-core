// Muscle.cpp
// Author: Peter Loan, Jeff Reinbolt
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
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/XMLNode.h>
#include "Muscle.h"
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include "ConditionalPathPoint.h"
#include "PointForceDirection.h"
#include "GeometryPath.h"
#include <OpenSim/Simulation/Wrap/PathWrapPoint.h>
#include <OpenSim/Simulation/Wrap/WrapResult.h>
#include <OpenSim/Simulation/Wrap/PathWrap.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include "Model.h"
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Common/DebugUtilities.h>
#include "SimTKsimbody.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

const int Muscle::STATE_ACTIVATION = 0;
const int Muscle::STATE_FIBER_LENGTH = 1;

static int counter=0;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Muscle::Muscle() : Actuator(),
	_pathProp(PropertyObj("", GeometryPath())),
	_path((GeometryPath&)_pathProp.getValueObj()),
   _defaultActivation(0),
   _defaultFiberLength(0)
{
	setNull();
	setupProperties();
	// override the value of default _minControl, _maxControl
	_minControl=0.0;
	_maxControl=1.0;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Muscle::~Muscle()
{
	VisibleObject* disp;
	if ((disp = getDisplayer())){
		 // Free up allocated geometry objects
		disp->freeGeometry();
	}
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMuscle Muscle to be copied.
 */
Muscle::Muscle(const Muscle &aMuscle) : Actuator(aMuscle),
	_pathProp(PropertyObj("", GeometryPath())),
	_path((GeometryPath&)_pathProp.getValueObj()),
   _defaultActivation(aMuscle._defaultActivation),
   _defaultFiberLength(aMuscle._defaultFiberLength)
{
	setNull();
	setupProperties();
	copyData(aMuscle);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one Muscle to another.
 *
 * @param aMuscle Muscle to be copied.
 */
void Muscle::copyData(const Muscle &aMuscle)
{
	_path = aMuscle._path;
	// This should be moved to a base class method for future changes
	//_minControl=aMuscle._minControl;
	//_maxControl=aMuscle._maxControl;
}

//_____________________________________________________________________________
/**
 * Set the data members of this Muscle to their null values.
 */
void Muscle::setNull()
{
	setType("Muscle");
	//_minControl=0.01;
	//_maxControl=1.0;
}

//_____________________________________________________________________________
/**
 * Override default implementation by object to intercept and fix the XML node
 * underneath the model to match current version
 */
void Muscle::updateFromXMLNode()
{
	int documentVersion = getDocument()->getDocumentVersion();
	if ( documentVersion < XMLDocument::getLatestVersion()){
		DOMDocument* document = _node->getOwnerDocument();
		if (Object::getDebugLevel()>=1)
			cout << "Updating Muscle object to latest format..." << endl;
		// Version has to be 1.6 or later, otherwise assert
		if (_node!=NULL){
			DOMElement* pathNode = XMLNode::GetFirstChildElementByTagName(_node, "GeometryPath");
			if (pathNode) {
				// Do nothing for now.
			} else {
				pathNode = XMLNode::CreateDOMElement(document, "GeometryPath");
				DOMElement* attachmentsNode = XMLNode::GetFirstChildElementByTagName(_node, "MusclePointSet");
				if (attachmentsNode != 0) {
					pathNode->appendChild(attachmentsNode->cloneNode(true));
					_node->insertBefore(pathNode, attachmentsNode);
					_node->removeChild(attachmentsNode);
				}
				DOMElement* displayerNode = XMLNode::GetFirstChildElementByTagName(_node, "display");
				if (displayerNode != 0) {
					pathNode->appendChild(displayerNode->cloneNode(true));
					_node->removeChild(displayerNode);
				}
				DOMElement* wrapNode = XMLNode::GetFirstChildElementByTagName(_node, "MuscleWrapSet");
				if (wrapNode != 0) {
					pathNode->appendChild(wrapNode->cloneNode(true));
					_node->removeChild(wrapNode);
				}
			}
			renameChildNode("MusclePointSet", "PathPointSet", pathNode);
			renameChildNode("MuscleWrapSet", "PathWrapSet", pathNode);
		}
	}
	// Call base class now assuming _node has been corrected for current version
	Actuator::updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aModel The model containing this muscle.
 */
void Muscle::setup(Model& aModel)
{
	// Specify underlying ModelComponents prior to calling base::setup() to automatically 
	// propogate setup to subcomponents. Subsequent createSystem() will also be automatically
	// propogated.
	includeAsSubComponent(&_path);
	Actuator::setup(aModel);

	// _model will be NULL when objects are being registered.
	if (_model == NULL)
		return;

    setNumStateVariables(2);

	_stateVariableSuffixes[STATE_ACTIVATION]="activation";
	_stateVariableSuffixes[STATE_FIBER_LENGTH]="fiber_length";

	_path.setOwner(this);
}

//_____________________________________________________________________________
/**
 * allocate and initialize the SimTK state for this acuator.
 */
 void Muscle::createSystem(SimTK::MultibodySystem& system) const
{
	Actuator::createSystem(system);

	Muscle* mutableThis = const_cast<Muscle *>(this);

	mutableThis->setNumStateVariables(_stateVariableSuffixes.getSize());
	mutableThis->addStateVariables(_stateVariableSuffixes);
	mutableThis->addCacheVariable<SimTK::Vector>("state_derivatives", SimTK::Vector(getNumStateVariables()), SimTK::Stage::Dynamics);

	mutableThis->_model->addModelComponent(this);
 }

 void Muscle::initState( SimTK::State& s) const
{
    Actuator::initState(s);

	Muscle* mutableThis = const_cast<Muscle *>(this);

	// keep track of the index for the first state variable derivatives in the cache 
	mutableThis->_zIndex = getZIndex(_stateVariableSuffixes[0]);

	setActivation(s, _defaultActivation);
	setFiberLength(s, _defaultFiberLength);
}

void Muscle::setDefaultsFromState(const SimTK::State& state)
{
	Actuator::setDefaultsFromState(state);
    _defaultActivation = getActivation(state);
    _defaultFiberLength = getFiberLength(state);
}

double Muscle::getDefaultActivation() const {
    return _defaultActivation;
}
void Muscle::setDefaultActivation(double activation) {
    _defaultActivation = activation;
}
double Muscle::getDefaultFiberLength() const {
    return _defaultFiberLength;
}
void Muscle::setDefaultFiberLength(double length) {
    _defaultFiberLength = length;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Muscle::setupProperties()
{
	_pathProp.setName("GeometryPath");
	_propertySet.append(&_pathProp);
}

//_____________________________________________________________________________
/**
 * Set the name of the muscle. This method overrides the one in Object
 * so that the path points can be [re]named accordingly.
 *
 * @param aName The new name of the muscle.
 */
void Muscle::setName(const string &aName)
{
	// base class
	Actuator::setName(aName);

	// Give the path the same name as the muscle so that print statements can use
	// it and so the path points can be named appropriately.
	_path.setName(aName);
}


//_____________________________________________________________________________
/**
 */
void Muscle::setNumStateVariables( int aNumStateVariables)
{
	_numStateVariables = aNumStateVariables;
	_stateVariableSuffixes.setSize(aNumStateVariables);
}

//_____________________________________________________________________________
/**
 * Get the name of a state variable, given its index.
 *
 * @param aIndex The index of the state variable to get.
 * @return The name of the state variable.
 */
string Muscle::getStateVariableName(int aIndex) const
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

// STATES
int Muscle::getNumStateVariables() const
{
	Muscle* mutableThis = const_cast<Muscle *>(this);
    return mutableThis->updRep()->getNumStateVariablesAddedByModelComponent(); //+ numStatesOfUnderlyingComponent
}

//_____________________________________________________________________________
/**
 * Set the derivative of an actuator state, specified by index
 *
 * @param aIndex The index of the state to set.
 * @param aValue The value to set the state to.
 */
void Muscle::setStateVariableDeriv(const SimTK::State& s, int aIndex, double aValue) const {

	SimTK::Vector& stateDeriv =  updCacheVariable<SimTK::Vector>(s, "state_derivatives");
	if(0<=aIndex && aIndex<_numStateVariables) {
		stateDeriv[aIndex] = aValue;
	} else {
		std::stringstream msg;
		msg << "Muscle::setStateVariableDeriv: ERR- index out of bounds.\nActuator " 
			 << getName() << " of type " << getType() << " has " << getNumStateVariables() << " states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
	markCacheVariableValid(s, "state_derivatives");
}

//_____________________________________________________________________________
/**
 * Get the derivative of an actuator state, by index.
 *
 * @param aIndex the index of the state to get.
 * @return The value of the state.
 */
double Muscle::getStateVariableDeriv(const SimTK::State& s, int aIndex) const
{
	const SimTK::Vector& stateDeriv = getCacheVariable<SimTK::Vector>(s, "state_derivatives");
	if(0<=aIndex && aIndex<_numStateVariables) {
        return( stateDeriv[aIndex] );
	} else {
		std::stringstream msg;
		msg << "MusclegetStateVariableDeriv: ERR- index out of bounds.\nActuator " 
		    << getName() << " of type " << getType() << " has " << getNumStateVariables() << " states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}




//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @param aMuscle The muscle from which to copy its data
 * @return Reference to this object.
 */
Muscle& Muscle::operator=(const Muscle &aMuscle)
{
	// base class
	Actuator::operator=(aMuscle);

	copyData(aMuscle);

	return(*this);
}


//=============================================================================
// GENERIC NORMALIZED FORCE-LENGTH-VELOCIY PROPERTIES
//=============================================================================
//_____________________________________________________________________________
/**
 * Evaluate the normalized force-length-velocity curve for the muscle.
 * A simple generic implementation is used here.  Derived classes should
 * override this method for more precise evaluation of the
 * force-length-velocity curve.
 *
 * @param aActivation Activation level of the muscle.  1.0 is full activation;
 * 0.0 is no activation.
 * @param aNormalizedLength Normalized length of the muscle fibers.  1.0 indicates
 * the muscle fibers are at their optimal length.  Lnorm = L / Lo.
 * @param aNormalizedVelocity Normalized shortening velocity of the muscle fibers.
 * Positive values indicate concentric contraction (shortening); negative values
 * indicate eccentric contraction (lengthening).  Normalized velocity is
 * the fiber shortening velocity divided by the maximum shortening velocity times
 * the optimal fiber length.  Vnorm = V / (Vmax*Lo).
 * @return Force normalized by the optimal force.
 */
double Muscle::
evaluateForceLengthVelocityCurve(double aActivation, double aNormalizedLength, double aNormalizedVelocity)
{
	// force-length
	double fLength = exp(-17.33 * fabs(pow(aNormalizedLength-1.0,3)));

	// force-velocity
	double fVelocity = 1.8  -  1.8 / (1.0 + exp( (0.04 - aNormalizedVelocity)/0.18) );

	return aActivation * fLength * fVelocity;
}


//=============================================================================
// GET
//=============================================================================
//-----------------------------------------------------------------------------
// LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the length of the muscle. This is a convenience function that passes
 * the request on to the muscle path.
 *
 * @return Current length of the muscle path.
 */
double Muscle::getLength(const SimTK::State& s) const
{
	return _path.getLength(s);
}

//_____________________________________________________________________________
/**
 * Get the length of the tendon.
 *
 * @return Current length of the tendon.
 */
double Muscle::getTendonLength(const SimTK::State& s) const
{
	return getLength(s) - getFiberLengthAlongTendon(s);
}
//_____________________________________________________________________________
/**
 * Get the length of the muscle fiber(s) along the tendon. This method
 * accounts for the pennation angle. 
 *
 * @return Current length of the muscle fiber(s) along the direction of
 * the tendon.
 */
double Muscle::getFiberLengthAlongTendon(const SimTK::State& s) const
{
	return getFiberLength(s) * cos(getPennationAngle(s));
}

//-----------------------------------------------------------------------------
// FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the force generated by the muscle fibers. This accounts for
 * pennation angle. That is, the fiber force is computed by dividing the
 * actuator force by the cosine of the pennation angle.
 *
 * @return Force in the muscle fibers.
 */
double Muscle::getFiberForce(const SimTK::State& s) const
{
	double force;
	double cos_penang = cos(getPennationAngle(s));
	if(fabs(cos_penang) < SimTK::Zero) {
		force = SimTK::NaN;
	} else {
		force = getForce(s) / cos_penang;
	}

	return force;
}
//_____________________________________________________________________________
/**
 * Get the active force generated by the muscle fibers.
 *
 * @return Current active force of the muscle fibers.
 */
double Muscle::getActiveFiberForce(const SimTK::State& s) const
{
	return getFiberForce(s) - getPassiveFiberForce(s);
}
//_____________________________________________________________________________
/**
 * Get the active force generated by the muscle fibers along the direction
 * of the tendon.
 *
 * @return Current active force of the muscle fibers along tendon.
 */
double Muscle::getActiveFiberForceAlongTendon(const SimTK::State& s) const
{
	return getActiveFiberForce(s) * cos(getPennationAngle(s));
}
//_____________________________________________________________________________
/**
 * Get the passive force generated by the muscle fibers along the direction
 * of the tendon.
 *
 * @return Current passive force of the muscle fibers along tendon.
 */
double Muscle::getPassiveFiberForceAlongTendon(const SimTK::State& s) const
{
	return getPassiveFiberForce(s) * cos(getPennationAngle(s));
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform computations that need to happen before the muscle is scaled.
 * For this object, that entails calculating and storing the muscle-tendon
 * length in the current body position.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void Muscle::preScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	_path.preScale(s, aScaleSet);
}

//_____________________________________________________________________________
/**
 * Scale the muscle based on XYZ scale factors for each body.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 * @return Whether muscle was successfully scaled or not.
 */
void Muscle::scale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	_path.scale(s, aScaleSet);
}

//_____________________________________________________________________________
/**
 * Perform computations that need to happen after the muscle is scaled.
 * For this object, that entails updating the muscle path. Derived classes
 * should probably also scale or update some of the force-generating
 * properties.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void Muscle::postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	_path.postScale(s, aScaleSet);
}

//--------------------------------------------------------------------------
// COMPUTATIONS
//--------------------------------------------------------------------------
/**
 * Compute the moment-arm of this muscle about a coordinate.
 */
double Muscle::computeMomentArm(SimTK::State& s, Coordinate& aCoord) const
{
	return _path.computeMomentArm(s, aCoord);
}


double Muscle::getLengtheningSpeed(const SimTK::State& s) const 
{
	return _path.getLengtheningSpeed(s);
}

//_____________________________________________________________________________
/**
 * Utility function to calculate the current pennation angle in a
 * muscle. Pennation angle increases as muscle fibers shorten. The implicit
 * modeling assumption is that muscles have constant width.
 *
 * @param aFiberLength Current fiber length of muscle.
 * @param aOptimalFiberLength Optimal fiber length of muscle.
 * @param aInitialPennationAngle Pennation angle at optimal fiber length (in radians).
 * @return Current pennation angle (in radians).
 */
double Muscle::calcPennation( double aFiberLength, double aOptimalFiberLength,
											    double aInitialPennationAngle) const
{
	if (aFiberLength < SimTK::Eps)
		return 0.0;

   double value = aOptimalFiberLength * sin(aInitialPennationAngle) / aFiberLength;

   if ( isnan(value)  ) 
       return 0.0;
   else if (value <= 0.0 )
      return 0.0;
   else if (value >= 1.0)
		return SimTK_PI/2.0;
   else
      return asin(value);
}

//=============================================================================
// FORCE APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply the muscle's force at its points of attachment to the bodies.
 */
void Muscle::computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const
{
	double muscleForce = 0;

	if( isForceOverriden(s) ) {
		muscleForce = computeOverrideForce(s);
		// Also define the state derivatives, since realize acceleration will
		// ask for muscle derivatives, which will be integrated
		// in the case the force is being overridden, the states aren't being used
		// but a valid derivative cache entry is still required 
		SimTK::Vector& stateDerivs =  updCacheVariable<SimTK::Vector>(s, "state_derivatives");
		stateDerivs = 0.0;
		markCacheVariableValid(s, "state_derivatives");
    } else {
       muscleForce = computeActuation(s);
    }
    setForce(s, muscleForce);

	// NOTE: Force could be negative, in particular during CMC, when the optimizer is computing
	// gradients, it will setForce(+1) and setForce(-1) to compute the derivative with respect to force.
	if (fabs( muscleForce ) < SimTK::SqrtEps) {
		//std::cout << "Muscle::computeForce muscleForce < SimTK::SqrtEps" << getName() << std::endl;
		return;
    }

	OpenSim::Array<PointForceDirection*> PFDs;
	_path.getPointForceDirections(s, &PFDs);

	for (int i=0; i < PFDs.getSize(); i++) {
		applyForceToPoint(s, PFDs[i]->body(), PFDs[i]->point(), muscleForce*PFDs[i]->direction(), bodyForces);
	}
	for(int i=0; i < PFDs.getSize(); i++)
		delete PFDs[i];
}

//_____________________________________________________________________________
/**
 * Get the visible object used to represent the muscle.
 */
VisibleObject* Muscle::getDisplayer() const
{ 
	return getGeometryPath().getDisplayer(); 
}

//_____________________________________________________________________________
/**
 * Update the visible object used to represent the muscle.
 */
void Muscle::updateDisplayer(const SimTK::State& s)
{
	_path.updateDisplayer(s);
}
//_____________________________________________________________________________
/**
 * getMaxIsometricForce needs to be overridden by derived classes to be usable
 */
double Muscle::getMaxIsometricForce() const
{
	OPENSIM_ERROR_IF_NOT_OVERRIDDEN();
}


/**
 * Estimate an new activation level given an initial activation level,
 * an excitation level, and a time interval.
 * The assumptions are that the excitation is constant over the interval and
 * that activation dynamics is represented as a pure exponential.
 * The equation for activation is
 *
 * 	at = x - (x-a0)*exp[-dt/tau]
 *
 * @param aTRise Activation rise time constant.
 * @param aTFall Activation fall time constant.
 * @param aA0 Starting value of activation.
 * @param aX Excitation value.
 * @param aDT Time interval over which activation is to change.
 * @return Estimated activation level.
 */
double Muscle::
EstimateActivation(double aTRise,double aTFall,double aA0,double aX,double aDT)
{
	// ERROR
	if(aDT<=0) {
		return(aA0);
	}

	// CHECK at==aA0
	if(aX==aA0) return(aA0);

	// DETERMINE TIME CONSTANT
	double tau;
	if(aX>=aA0) {
		tau = aTRise;
	} else {
		tau = aTFall;
	}

	// CHECK FOR ZERO TAU
	if(tau<=0) {
		printf("Muscle.EstimateActivation: ERROR- tau<=0.0\n");
		return(aA0);
	}

	// EXPONENTIAL
	double T = exp(-aDT/tau);

	// COMPUTE EXCITATION
	double a = aX - (aX-aA0)*T;

	return(a);
}
//_____________________________________________________________________________
/**
 * Invert the equation for activation dynamics in order to compute an
 * excitation value which will produce a given change in activation
 * over a given time interval.
 * The assumptions are that the excitation is constant over the interval and
 * that activation dynamics is represented as a pure exponential.
 * The equation which is inverted is
 *
 * 	at = x - (x-a0)*exp[-dt/tau]
 *
 * Parameters:
 * @param aTRise The rise time constant.
 * @param aTFall The fall time constant.
 * @param aA0 The starting value of activation
 * @param aA The final desired value of activation.
 * @param aDT The time interval over which a is to change.
 * @return Excitation that will achieve the desired activation.
 */
double Muscle::
InvertActivation(double aTRise,double aTFall,double aA0,double aA,double aDT)
{
	// ERROR
	if(aDT<=0) {
		printf("Muscle.invertActivation: ERROR- aDT<=0.0\n");
		return(aA);
	}

	// CHECK at==aA0
	if(aA==aA0) return(aA);

	// DETERMINE TIME CONSTANT
	double tau;
	if(aA>=aA0) {
		tau = aTRise;
	} else {
		tau = aTFall;
	}

	// CHECK FOR ZERO TAU
	if(tau<=0) {
		printf("Muscle.invertActivation: ERROR- tau<=0.0\n");
		return(aA);
	}

	// EXPONENTIAL
	double T = exp(-aDT/tau);

	// COMPUTE EXCITATION
	double x = (aA - aA0*T)/(1.0-T);

	return(x);
}
//_____________________________________________________________________________
/**
 * Compute the time derivative of an activation level given its excitation
 * signal, a rise-time, and a fall-time.
 * This method represents the rise or fall using a simple 1st order
 * differential equation which is linear in x and a.  The time constant is
 * chosen based on whether x is greater than or less than a.
 * 
 */
double Muscle::
DADT(double aTRise,double aTFall,double aX,double aA)
{
	// DETERMINE TIME CONSTANT
	double tau;
	if(aX>=aA) {
		tau = aTRise;
	} else {
		tau = aTFall;
	}

	// CHECK FOR ZERO TAU
	if(tau<=0) {
		printf("Muscle.dadt: ERROR- tau<=0.0\n");
		return(aX-aA);
	}

	// COMPUTE DERIVATIVE
	double dadt = (aX-aA)/tau;

	return(dadt);
}
//_____________________________________________________________________________
/**
 * Compute the time derivative of an activation level given its excitation
 * signal, a rise-time, and a fall-time.
 * This method represents the rise and fall using a 1st order differential
 * equation which is non-linear in x.  The advantange of this method is that
 * a single equation is used.  However, the equation is only valid if tFall
 * is mutch greater than tRise.
 */
double Muscle::
DADTNonlinear(double aTRise,double aTFall,double aX,double aA)
{
	double dadt = (aX*aX-aX*aA)/aTRise  + (aX-aA)/aTFall;
	return(dadt);
}




//=============================================================================
// MUSCLE MECHANICS AND DYNAMICS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the force in an actuator given its maximum force and activation
 * state.
 */
double Muscle::
f(double aFMax,double aA)
{
	double f = aA*aFMax;
	return(f);
}

void Muscle::updateGeometry(const SimTK::State& s) const
{
	_path.updateGeometry(s);
}

//_____________________________________________________________________________
//**
// * get the excitation value for this Muscle 
// */
double Muscle::
getExcitation( const SimTK::State& s) const {
    return( getControl(s) );
}
//_____________________________________________________________________________
/**
 * Add a Muscle point to the _path of the muscle. The new point is appended 
 * to the end of the current path
 *
 */
void Muscle::addNewPathPoint(
		 const std::string& proposedName, 
		 OpenSim::Body& aBody, 
		 const SimTK::Vec3& aPositionOnBody) {
	// Create new PathPoint
	PathPoint* newPathPoint =_path.appendNewPathPoint(proposedName, aBody, aPositionOnBody);
	// Set offset/position on owner body
	newPathPoint->setName(proposedName);
	for (int i=0; i<3; i++)	// Use interface that does not depend on state
		newPathPoint->setLocationCoord(i, aPositionOnBody[i]);
}

void Muscle::equilibrate(SimTK::State& state) const
{
	// Reasonable initial activation value
	setActivation(state, 0.01);
	setFiberLength(state, getOptimalFiberLength());
	_model->getMultibodySystem().realize(state, SimTK::Stage::Velocity);

	// Compute isometric force to get starting value
	// of _fiberLength.
	computeEquilibrium(state);
}