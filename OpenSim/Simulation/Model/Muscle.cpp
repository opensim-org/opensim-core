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

static int counter=0;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Muscle::Muscle() : PathActuator()
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
Muscle::Muscle(const Muscle &aMuscle) : PathActuator(aMuscle)
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
}

//_____________________________________________________________________________
/**
 * Set the data members of this Muscle to their null values.
 */
void Muscle::setNull()
{
	setType("Muscle");
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
	PathActuator::setup(aModel);

	// _model will be NULL when objects are being registered.
	if (_model == NULL)
		return;
}

//_____________________________________________________________________________
/**
 * allocate and initialize the SimTK state for this acuator.
 */
 void Muscle::createSystem(SimTK::MultibodySystem& system) const
{
	PathActuator::createSystem(system);

 }

 void Muscle::initState( SimTK::State& s) const
{
    PathActuator::initState(s);
}

void Muscle::setDefaultsFromState(const SimTK::State& state)
{
	PathActuator::setDefaultsFromState(state);
}


//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Muscle::setupProperties()
{
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
	PathActuator::setName(aName);
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
	PathActuator::operator=(aMuscle);

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
double Muscle::evaluateForceLengthVelocityCurve(double aActivation, double aNormalizedLength, double aNormalizedVelocity) const
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

		// compute path's lengthening speed if necessary
	double speed = _path.getLengtheningSpeed(s);

	// the lengthening speed of this actutor is the "speed" of the actuator used to compute power
	setSpeed(s, speed);

	if( isForceOverriden(s) ) {
		muscleForce = computeOverrideForce(s);
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
 * getMaxIsometricForce needs to be overridden by derived classes to be usable
 */
double Muscle::getMaxIsometricForce() const
{
	OPENSIM_ERROR_IF_NOT_OVERRIDDEN();
}


void Muscle::updateGeometry(const SimTK::State& s) const
{
	_path.updateGeometry(s);
}