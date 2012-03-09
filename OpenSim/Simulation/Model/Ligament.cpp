// Ligament.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS: Peter Loan
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
 * Copyright (c)  2009, Stanford University. All rights reserved. 
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
#include "Ligament.h"
#include "GeometryPath.h"
#include "PointForceDirection.h"
#include <OpenSim/Common/NaturalCubicSpline.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTANTS
//=============================================================================

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Ligament::Ligament() : Force()
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 * Delete any variables allocated using the "new" operator.  You will not
 * necessarily have any of these.
 */
Ligament::~Ligament()
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
 * @param aLigament Ligament to be copied.
 */
Ligament::Ligament(const Ligament &aLigament) : Force(aLigament)
{
	setNull();
	setupProperties();
	copyData(aLigament);
}

//_____________________________________________________________________________
/**
 * Copy this ligament and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this ligament.
 */
Object* Ligament::copy() const
{
	Ligament *lig = new Ligament(*this);
	return lig;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one ligament to another.
 *
 * @param aLigament Ligament to be copied.
 */
void Ligament::copyData(const Ligament &aLigament)
{
	setPropertyValue("GeometryPath", aLigament.getPropertyValue<GeometryPath>("GeometryPath"));
	setPropertyValue("resting_length", aLigament.getPropertyValue<double>("resting_length"));
	setPropertyValue("pcsa_force", aLigament.getPropertyValue<double>("pcsa_force"));
	setPropertyValue("force_length_curve", (Function*)Object::SafeCopy(aLigament.getPropertyValue<Function *>("force_length_curve")));
}

//_____________________________________________________________________________
/**
 * Set the data members of this ligament to their null values.
 */
void Ligament::setNull()
{
	setType("Ligament");

	_model = NULL;
}

//_____________________________________________________________________________
/**
 * Set up the properties for the ligament.
 * 
 * You should give each property a meaningful name and an informative comment.
 * The name you give each property is the tag that will be used in the XML
 * file. The comment will appear before the property in the XML file.
 * In addition, the comments are used for tool tips in the OpenSim GUI.
 *
 * All properties are added to the property set. Once added, they can be
 * read in and written to file.
 */
void Ligament::setupProperties()
{
	addProperty<GeometryPath>("GeometryPath",
		"the set of points defining the path of the ligament",
		GeometryPath());
	addProperty<double>("resting_length",
		"resting length of the ligament",
		0.0);
	addProperty<double>("pcsa_force",
		"force magnitude that scales the force-length curve",
		0.0);
	int forceLengthCurvePoints = 13;
	double forceLengthCurveX[] = {-5.00000000,  0.99800000,  0.99900000,  1.00000000,  1.10000000,  1.20000000,  1.30000000,  1.40000000,  1.50000000,  1.60000000,  1.60100000,  1.60200000,  5.00000000};
	double forceLengthCurveY[] = {0.00000000,  0.00000000,  0.00000000,  0.00000000,  0.03500000,  0.12000000,  0.26000000,  0.55000000,  1.17000000,  2.00000000,  2.00000000,  2.00000000,  2.00000000};
	NaturalCubicSpline *forceLengthCurve = new NaturalCubicSpline(forceLengthCurvePoints, forceLengthCurveX, forceLengthCurveY);
	addProperty<Function *>("force_length_curve",
		"Function representing the force-length behavior of the ligament",
		forceLengthCurve);
}

//_____________________________________________________________________________
/**
 * Perform some setup functions that happen after the
 * ligament has been deserialized or copied.
 *
 * @param aModel model containing this ligament.
 */
void Ligament::setup(Model& aModel)
{
	GeometryPath &path = updPropertyValue<GeometryPath>("GeometryPath");
	const double &restingLength = getPropertyValue<double>("resting_length");

	// Specify underlying ModelComponents prior to calling base::setup() to automatically 
	// propogate setup to subcomponents. Subsequent createSystem() will also be automatically
	// propogated to subcomponents.
	includeAsSubComponent(&path);
	Force::setup(aModel);

	// _model will be NULL when objects are being registered.
	if (_model == NULL)
		return;

	// Resting length must be greater than 0.0.
	assert(restingLength > 0.0);

	path.setOwner(this);
}

//_____________________________________________________________________________
/**
 * allocate and initialize the SimTK state for this ligament.
 */
 void Ligament::createSystem(SimTK::MultibodySystem& system) const
{
	Force::createSystem(system);
}


void Ligament::initState( SimTK::State& s) const
{
	Force::initState(s);
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this ligament.
 */
Ligament& Ligament::operator=(const Ligament &aLigament)
{
	// BASE CLASS
	Force::operator=(aLigament);

	copyData(aLigament);

	return *this;
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the length of the ligament. This is a convenience function that passes
 * the request on to the ligament path.
 *
 * @return Current length of the ligament path.
 */
double Ligament::getLength(const SimTK::State& s) const
{
	return getPropertyValue<GeometryPath>("GeometryPath").getLength(s);
}

//_____________________________________________________________________________
/**
 * Set the resting length.
 *
 * @param aRestingLength The resting length of the ligament.
 * @return Whether the resting length was successfully changed.
 */
bool Ligament::setRestingLength(double aRestingLength)
{
	setPropertyValue("resting_length", aRestingLength);
	return true;
}

//_____________________________________________________________________________
/**
 * Set the maximum isometric force.
 *
 * @param aMaxIsometricForce The maximum isometric force of the ligament.
 * @return Whether the maximum isometric force was successfully changed.
 */
bool Ligament::setMaxIsometricForce(double aMaxIsometricForce)
{
	setPropertyValue("pcsa_force", aMaxIsometricForce);
	return true;
}

//_____________________________________________________________________________
/**
 * Set the force-length curve.
 *
 * @param aForceLengthCurve Pointer to a force-length curve (Function).
 * @return Whether the force-length curve was successfully changed.
 */
bool Ligament::setForceLengthCurve(Function* aForceLengthCurve)
{
	setPropertyValue("force_length_curve", aForceLengthCurve);
	return true;
}
//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform computations that need to happen before the ligament is scaled.
 * For this object, that entails calculating and storing the
 * length in the current body position.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void Ligament::preScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	updPropertyValue<GeometryPath>("GeometryPath").preScale(s, aScaleSet);
}

//_____________________________________________________________________________
/**
 * Scale the ligament.
 *
 * @param aScaleSet XYZ scale factors for the bodies
 * @return Whether or not the ligament was scaled successfully
 */
void Ligament::scale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	updPropertyValue<GeometryPath>("GeometryPath").scale(s, aScaleSet);
}

//_____________________________________________________________________________
/**
 * Perform computations that need to happen after the ligament is scaled.
 * For this object, that entails comparing the length before and after scaling,
 * and scaling the resting length a proportional amount.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void Ligament::postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	GeometryPath &path = updPropertyValue<GeometryPath>("GeometryPath");
	double &restingLength = updPropertyValue<double>("resting_length");

	path.postScale(s, aScaleSet);

	if (path.getPreScaleLength(s) > 0.0)
	{
		double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);

		// Scale resting length by the same amount as the change in
		// total ligament length (in the current body position).
		restingLength *= scaleFactor;

		path.setPreScaleLength(s, 0.0);
	}
}

//=============================================================================
// COMPUTATION
//=============================================================================
/**
 * Compute the moment-arm of this muscle about a coordinate.
 */
double Ligament::computeMomentArm(SimTK::State& s, Coordinate& aCoord) const
{
	return getGeometryPath().computeMomentArm(s, aCoord);
}



void Ligament::computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const
{
	const GeometryPath &path = getPropertyValue<GeometryPath>("GeometryPath");
	const double &restingLength = getPropertyValue<double>("resting_length");
	const double &pcsaForce = getPropertyValue<double>("pcsa_force");

	if (path.getLength(s) <= restingLength)
		return;

	double strain = (path.getLength(s) - restingLength) / restingLength;
	double force = getForceLengthCurve()->calcValue(SimTK::Vector(1, strain)) * pcsaForce;

	OpenSim::Array<PointForceDirection*> PFDs;
	path.getPointForceDirections(s, &PFDs);

	for (int i=0; i < PFDs.getSize(); i++) {
		applyForceToPoint(s, PFDs[i]->body(), PFDs[i]->point(), force*PFDs[i]->direction(), bodyForces);
	}
}

//_____________________________________________________________________________
/**
 * Get the visible object used to represent the Ligament.
 */
VisibleObject* Ligament::getDisplayer() const
{ 
	return getGeometryPath().getDisplayer(); 
}

//_____________________________________________________________________________
/**
 * Update the visible object used to represent the Ligament.
 */
void Ligament::updateDisplayer(const SimTK::State& s)
{
	updGeometryPath().updateDisplayer(s);
}