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
#include <OpenSim/Common/IO.h>
#include <SimTKcommon/internal/Exception.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Transform.h>
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

#ifdef SWIG
	#ifdef OSIMMIGRATESIMMKEMODEL_API
		#undef OSIMMIGRATESIMMKEMODEL_API
		#define OSIMMIGRATESIMMKEMODEL_API
	#endif
#endif

using namespace std;
using namespace OpenSim;

#define ROUNDOFF_ERROR 0.000000001
#define DABS(a) ((a)>(double)0.0?(a):(-(a)))
#define NOT_EQUAL_WITHIN_ERROR(a,b) (DABS(((a)-(b))) > ROUNDOFF_ERROR)

bool isJointFixed(SimmJoint& aJoint);
void convert_dof_to_function(AbstractDof01_05* dof, string coordinateName);
void extract_xyz_rot_bodyfixed(double m[], double xyz_rot[3]);
void extract_joint_locations_and_orientations(SimmJoint* joint,
                                              double locationInParent[],
                                              double orientationInParent[],
                                              double locationInChild[],
                                              double orientationInChild[]);


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
		newBody->setJoint(newJoint);
		delete newJoint;  // A copy is made, so must delete original.
		newJoint = newBody->getJoint();
		if (newJoint!=NULL)
			jointSet->append(newJoint);
		// Name
		newJoint->setName(oldJoint->getName());
		// Parent Body Name
		newJoint->setParentName(oldJoint->getParentBodyName());

		double locationInParent[3], orientationInParent[3], locationInChild[3], orientationInChild[3];
		extract_joint_locations_and_orientations(oldJoint,
			locationInParent, orientationInParent,
			locationInChild, orientationInChild);

		SimTK::Vec3 lp(locationInParent[0], locationInParent[1], locationInParent[2]);
		newJoint->setLocationInParent(lp);
		SimTK::Vec3 op(orientationInParent[0], orientationInParent[1], orientationInParent[2]);
		newJoint->setOrientationInParent(op);
		SimTK::Vec3 lc(locationInChild[0], locationInChild[1], locationInChild[2]);
		newJoint->setLocation(lc);
		SimTK::Vec3 oc(orientationInChild[0], orientationInChild[1], orientationInChild[2]);
		newJoint->setOrientation(oc);

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

			if(function->getType() != "Constant") {
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

				// Does the coordinate already exist?
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

// This function deals with the constant translations and rotations that are in a joint.
// Simbody does not allow them in DOFs, so they are handled as follows:
//   1. The string of constants at the beginning of the DOF list are converted into
//      locationInParent and orientationInParent.
//   2. The string of constants at the end of the DOF list are converted into
//      locationInChild and orientationInChild.
//   3. The constants in between functions are turned into functions that are
//      constrained to remain at the proper value.
void extract_joint_locations_and_orientations(SimmJoint* joint,
                                              double locationInParent[],
                                              double orientationInParent[],
                                              double locationInChild[],
                                              double orientationInChild[])
{
	const DofSet01_05 *dofSet = joint->getDofSet();
	int na = dofSet->getSize();
   int i, first_function = na, last_function = na;
   double *dof_value;

	dof_value = new double [na];

   for (i=0; i<na; i++) {
		AbstractDof01_05* dof = dofSet->get(i);
		if (dof && dof->getFunction()) {
			if (dof->getFunction()->getType() != "Constant") {
				first_function = i;
				break;
			}
		}
	}

   for (i=na-1; i>=0; i--) {
		AbstractDof01_05* dof = dofSet->get(i);
		if (dof && dof->getFunction()) {
			if (dof->getFunction()->getType() != "Constant") {
				last_function = i;
				break;
			}
		}
	}

   // Constants that are in between functions are converted to functions.
   // The gencoord used for these converted constants is the
   // 'first_function' gencoord.
   for (i=first_function+1; i<last_function; i++)
   {
		AbstractDof01_05* dof = dofSet->get(i);
		if (dof && dof->getFunction() && dof->getFunction()->getType() == "Constant" && NOT_EQUAL_WITHIN_ERROR(dof->getValue(), 0.0))
         convert_dof_to_function(dof, dofSet->get(first_function)->getCoordinateName());
   }

	// Make a temporary joint that consists of just the constant
	// dofs at the beginning of the joint 'joint.' This is done
	// by copying 'joint' and then setting the dofs at the end
	// to have functions of constant=0.0.
	SimmJoint parentSide(*joint);
	//Constant zero;
	for (i=first_function; i<na; i++) {
		Constant* zero = new Constant();
		parentSide.getDofSet()->get(i)->setFunction(zero);
	}

	// Now get the forward transform, which contains the matrix equivalent
	// of locationInParent and orientationInParent.
	Transform parentTransform = parentSide.getForwardTransform();

	// Make a temporary joint that consists of just the constant
	// dofs at the end of the joint 'joint.' This is done
	// by copying 'joint' and then setting the dofs at the beginning
	// to have functions of constant=0.0.
	SimmJoint childSide(*joint);
	if (last_function == na) // to handle case where all DOFs are constant
		last_function--;
	for (i=0; i<=last_function; i++) {
		Constant* zero = new Constant();
		childSide.getDofSet()->get(i)->setFunction(zero);
	}

	// Now get the inverse transform, which contains the matrix equivalent
	// of locationInChild and orientationInChild.
	Transform childTransform = childSide.getInverseTransform();

   // Extract the translations from the matrices.
	parentTransform.getPosition(locationInParent);
	childTransform.getPosition(locationInChild);

   // Extract the rotations from the matrices.
   extract_xyz_rot_bodyfixed(parentTransform.getMatrix(), orientationInParent);
   extract_xyz_rot_bodyfixed(childTransform.getMatrix(), orientationInChild);
}

void convert_dof_to_function(AbstractDof01_05* dof, string coordinateName)
{
	double x[] = {0.0, 1.0};
	double y[2];

	y[0] = y[1] = dof->getValue();

	NatCubicSpline* func = new NatCubicSpline(2, x, y);

	dof->setCoordinateName(coordinateName);
	dof->setFunction(func);
}

void extract_xyz_rot_bodyfixed(double m[], double xyz_rot[3])
{
   /* NOTE: extracts BODY-FIXED rotations in x,y,z order, which
    *  is the same as space-fixed rotations in z,y,x order.
    */
   xyz_rot[1] = asin(m[8]);
   
   if (NOT_EQUAL_WITHIN_ERROR(0.0, cos(xyz_rot[1])))
   {
      xyz_rot[0] = atan2(-m[9], m[10]);
      xyz_rot[2] = atan2(-m[4], m[0]);
   }
   else
   {
      xyz_rot[0] = atan2(m[1], m[5]);
      xyz_rot[2] = 0.0;
   } 
   /* NOTE: a body-fixed sequence of rotations is equivalent to
    *  the same sequence of space-fixed rotation in the *opposite*
    *  order!!  (see: "Introduction to Robotics, 2nd Ed. by J. Craig,
    *  page 49)  -- KMS 2/17/99
    */
}
