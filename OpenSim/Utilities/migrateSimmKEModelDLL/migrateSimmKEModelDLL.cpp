/* Copyright (c)  2008, Stanford University, Ajay Seth, and Peter Loan.
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

// INCLUDES
#include <string>
#include <OpenSim/version.h>
#include <OpenSim/Common/IO.h>
#include <SimTKcommon/internal/Exception.h>
#include <OpenSim/Common/rdMath.h>
#include "migrateSimmKEModelDLL.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/JointSet.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/WeldJoint.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/CustomJoint.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/TransformAxis.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/CoordinateCouplerConstraint.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmKinematicsEngine.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmBody.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmJoint.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmCoordinate.h>
#include <OpenSim/Actuators/Schutte1993Muscle.h>
#include <OpenSim/Actuators/Thelen2003Muscle.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>

#ifdef SWIG
	#ifdef OSIMMIGRATESIMMKEMODEL_API
		#undef OSIMMIGRATESIMMKEMODEL_API
		#define OSIMMIGRATESIMMKEMODEL_API
	#endif
#endif

using namespace std;
using namespace OpenSim;

bool isJointFixed(SimmJoint& aJoint);

OSIMMIGRATESIMMKEMODEL_API AbstractDynamicsEngine* OpenSim::makeSimbodyEngine(Model& aModel, const SimmKinematicsEngine& aSimmEngine)
{
	bool dynamicsReady = true;
	SimbodyEngine* sbe = new SimbodyEngine();

	// LOOP THROUGH BODIES
	CoordinateSet tmpGlobalCoordSet;
	tmpGlobalCoordSet.setMemoryOwner(false);
	BodySet *newBodySet = sbe->getBodySet();
	const BodySet *oldBodySet = aSimmEngine.getBodySet();
	int nb = oldBodySet->getSize();
	for(int i=0;i<nb;i++) {

		OpenSim::Body *newBody = new Body();
		SimmBody *oldBody = dynamic_cast<SimmBody*>( oldBodySet->get(i) );
		if(oldBody==NULL) continue;
		
		// BASE CLASS ASSIGNMENT OPERATOR
		// We use the assignment operator to get the name, visual properties, etc.
		// Problem is that it overwrites the type as well!	-Ayman 7/08
		std::string saveType = newBody->getType();
		(*newBody).AbstractBody::operator =(*oldBody);
		newBody->setType(saveType);
		newBody->setDynamicsEngine(NULL);

		// INERTIAL PROPERTIES
		// mass
		double mass = oldBody->getMass();
		newBody->setMass(mass);
		if (newBody->getName() != "ground" && mass <= 0.0) {
			cerr << "Mass of " << newBody->getName() << " is <= 0.0." << endl;
			dynamicsReady = false;
		}
		// mass center
		SimTK::Vec3 massCenter;
		oldBody->getMassCenter(massCenter);
		newBody->setMassCenter(massCenter);
		// inertia
		SimTK::Mat33 inertia;
		oldBody->getInertia(inertia);
		newBody->setInertia(inertia);
		if (newBody->getName() != "ground" &&
			(inertia[0][0] <= 0.0 || inertia[1][1] <= 0.0 || inertia[2][2] <= 0.0)) {
			cerr << "Inertia of " << newBody->getName() << " is <= 0.0." << endl;
			dynamicsReady = false;
		}

		// display stuff
		newBody->setDisplayer(*oldBody->getDisplayer());
		newBody->setDynamicsEngine(sbe);

		// ADD BODY TO SET
		newBodySet->append(newBody);

		// FIND THE OLD JOINT FOR THIS BODY
		const JointSet *oldJointSet = aSimmEngine.getJointSet();
		int nj = oldJointSet->getSize();
		SimmJoint *oldJoint = NULL;
		for(int j=0;j<nj;j++) {
			SimmJoint *joint = (SimmJoint*) oldJointSet->get(j);
			if(joint==NULL) continue;
			if(newBody->getName() == joint->getBodyName()) {
				oldJoint = joint;
				break;
			}
		}
		if(oldJoint==NULL) continue;

		// CREATE A NEW JOINT
		JointSet* jointSet = sbe->getJointSet();
		Joint* newJoint;
		if (isJointFixed(*oldJoint) == true)
			newJoint = new WeldJoint();
		else
			newJoint = new CustomJoint();
		newBody->setJoint(newJoint);  delete newJoint;  // A copy is made, so must delete original.
		newJoint = (CustomJoint*)newBody->getJoint();
		if (newJoint!=NULL)
			jointSet->append(newJoint);
		// Name
		newJoint->setName(oldJoint->getName());
		// Parent Body Name
		newJoint->setParentName(oldJoint->getParentBodyName());
		// Location in Parent
		SimTK::Vec3 zeroVec(0.0, 0.0, 0.0);
		newJoint->setLocationInParent(zeroVec);
		// Location in Child
		newJoint->setLocation(zeroVec);
		newJoint->setDynamicsEngine(sbe);

		// FIND THE OLD DOFs FOR THIS JOINT
		TransformAxisSet *newTransformAxisSet = newJoint->getTransformAxisSet();
		const DofSet01_05 *oldDofSet = oldJoint->getDofSet();
		int na = oldDofSet->getSize();
		for(int a=0;a<na;a++) {

			AbstractDof01_05 *oldDof = oldDofSet->get(a);
			if(oldDof==NULL) continue;
			Function *function = oldDof->getFunction();
			if(function==NULL) continue;

			// Constant
			if(function->getType() == "Constant") {

				// Add constant translations to the location_in_parent;
				double c = function->evaluate(0,0.0);
				SimTK::Vec3 shift(0,0,0);
				if(oldDof->getName()=="tx") {
					shift[0] = c;
				} else if(oldDof->getName()=="ty") {
					shift[1] = c;
				} else if(oldDof->getName()=="tz") {
					shift[2] = c;
				}
				SimTK::Vec3 location;
				newJoint->getLocationInParent(location);
				location += shift;
				newJoint->setLocationInParent(location);

			// Not constant
			} else {

				// Axis
				TransformAxis *newAxis = new TransformAxis();
				newAxis->setName(oldDof->getName());
				SimTK::Vec3 axis(0,0,0);
				oldDof->getAxis(axis);
				newAxis->setAxis(axis);

				// Motion type (rotation or translation)
				string type = oldDof->getType();
				if(type == "SimmTranslationDof") {
					newAxis->setIsRotation(false);
				} else if(type == "SimmRotationDof") {
					newAxis->setIsRotation(true);
				}

				// Coordinates
				string coordinateName = oldDof->getCoordinateName();
				newAxis->setCoordinateName(coordinateName);
				const CoordinateSet *oldCoordSet = aSimmEngine.getCoordinateSet();
				const SimmCoordinate *oldCoord = (const SimmCoordinate*)oldCoordSet->get(newAxis->getCoordinateName());

				// Does the coordinate alread exist?
				Coordinate *newCoord = (Coordinate*)tmpGlobalCoordSet.get(coordinateName);

				// Add a New Coordinate
				if(newCoord==NULL) {
					newCoord = new Coordinate();
					newCoord->setName(coordinateName);
					newCoord->setJoint(newJoint);
					newCoord->setDefaultValue(oldCoord->getDefaultValue());
					newCoord->setInitialValue(oldCoord->getValue());
					newCoord->setTolerance(oldCoord->getTolerance());
					newCoord->setStiffness(oldCoord->getStiffness());
					newCoord->setRangeMin(oldCoord->getRangeMin());
					newCoord->setRangeMax(oldCoord->getRangeMax());
					newCoord->setKeys(oldCoord->getKeys());
					newCoord->setClamped(oldCoord->getClamped());
					newCoord->setLocked(oldCoord->getLocked());
					newCoord->setRestraintActive(false);
					newCoord->setDynamicsEngine(sbe);
					newJoint->getCoordinateSet()->append(newCoord);
					tmpGlobalCoordSet.append(newCoord);
				}

				// New Coordinate AND New Constraint
				// When the joint is different, this means that the coordinate belongs to a
				// different joint already.  In this case, a new coordinate needs to be made,
				// adding a new degree of freedom, and then adding a constraint.
				// The coordinateby appending "_constrained" to the 
				if(newCoord->getJoint() != newJoint) {
					
					// New Coordinate
					Coordinate *constrainedCoord = new Coordinate();
					string constrainedCoordName = newBody->getName() + "_" + newAxis->getName() + "_constrained";
					constrainedCoord->setName(constrainedCoordName);
					constrainedCoord->setJoint(newJoint);
					newJoint->getCoordinateSet()->append(constrainedCoord);
					// Set the name of the coordinate in the transform axis to the new coordinate.
					newAxis->setCoordinateName(constrainedCoordName);

					// New CoordinateCouplerConstraint
					CoordinateCouplerConstraint *constraint = new CoordinateCouplerConstraint();
					OpenSim::Array<string> indepCoordNames;
					indepCoordNames.append(coordinateName);
					constraint->setIndependentCoordinateNames(indepCoordNames);
					constraint->setDependentCoordinateName(constrainedCoordName);
					constraint->setFunction((Function*)function->copy());
					sbe->getConstraintSet()->append(constraint);

				} else {

					// Add Function to TransformAxis
					// Linear functions with a slope of 1 and intercept of 0 do not get
					// added to the transform axis.
					const double equalityTolerance = 1.0e-6;
					double mx,my,mz;
					function->isLinear(1.0e-3,-1,1,mx,-1,1,my,-1,1,mz);
					double intercept = function->evaluate(0,0.0);
					if(!rdMath::IsEqual(mx,1.0,equalityTolerance) || (!rdMath::IsEqual(intercept,0.0,equalityTolerance))) {
						newAxis->setFunction((Function*)function->copy());
					}
				}

				newTransformAxisSet->append(newAxis);
			}
		}
	}

	sbe->setup(&aModel);

	// Copy marker set as well.
	sbe->replaceMarkerSet(*((MarkerSet*)aSimmEngine.getMarkerSet()));

	if (dynamicsReady == false) {
		cerr << "WARNING: You will not be able to use " << aModel.getName() << " for dynamic simulations" << endl;
		cerr << "         because one or more inertial properties are less than or equal to zero." << endl;
	}

	return sbe;
}

bool isJointFixed(SimmJoint& aJoint)
{
	const DofSet01_05 *dofSet = aJoint.getDofSet();
	for (int i=0; i<dofSet->getSize(); i++) {
		AbstractDof01_05 *dof = dofSet->get(i);
		if (dof == NULL)
			continue;
		Function *function = dof->getFunction();
		if (function == NULL)
			continue;
		if (function->getType() != "Constant")
			return false;
	}
	return true;
}
