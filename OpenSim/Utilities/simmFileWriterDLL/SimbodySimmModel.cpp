// SimbodySimmModel.cpp
// Authors: Peter Loan
/*
 * Copyright (c)  2008, Stanford University. All rights reserved. 
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
#include <iostream>
#include <string>
#include <math.h>
#include <float.h>
#include <time.h>

#include <OpenSim/DynamicsEngines/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/CoordinateCouplerConstraint.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Common/NatCubicSpline.h>

#define ROUNDOFF_ERROR 0.000000001
#define DABS(a) ((a)>(double)0.0?(a):(-(a)))
#define EQUAL_WITHIN_ERROR(a,b) (DABS(((a)-(b))) <= ROUNDOFF_ERROR)
#define NOT_EQUAL_WITHIN_ERROR(a,b) (DABS(((a)-(b))) > ROUNDOFF_ERROR)

#include "SimbodySimmModel.h"


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodySimmModel::~SimbodySimmModel()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimbodySimmModel::SimbodySimmModel()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Constructor from a Simbody engine.
 */
SimbodySimmModel::SimbodySimmModel(const SimbodyEngine& aEngine)
{
	setNull();
   setup(aEngine);
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
SimbodySimmModel::SimbodySimmModel(const SimbodySimmModel& aModel)
{
	setNull();
   copyData(aModel);
}

//_____________________________________________________________________________
/**
 * Copy this engine and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimbodySimmModel.
 */
Object* SimbodySimmModel::copy() const
{
	SimbodySimmModel *object = new SimbodySimmModel(*this);
	return object;
}

//_____________________________________________________________________________
/**
 * Copy data members from one SimbodySimmModel to another.
 *
 * @param aModel SimbodySimmModel to be copied.
 */
void SimbodySimmModel::copyData(const SimbodySimmModel &aModel)
{
   // TODO
}

//_____________________________________________________________________________
/**
 * Set NULL values for all the variable members of this class.
 */
void SimbodySimmModel::setNull()
{
	setType("SimbodySimmModel");
   _engine = NULL;
   _maxFunctionUserNumber = -1;
}

//_____________________________________________________________________________
/**
 * Setup the SimbodySimmModel with information from a SimbodyEngine.
 *
 * @param aEngine The engine to make this SimbodySimmModel from.
 */
void SimbodySimmModel::setup(const SimbodyEngine& aEngine)
{
   _engine = &aEngine;

   // Make a simple linear function for unconstrained coordinates
   NatCubicSpline* f = new NatCubicSpline();
   f->addPoint(-2.0*SimTK::Pi, -2.0*SimTK::Pi);
   f->addPoint(2.0*SimTK::Pi, 2.0*SimTK::Pi);
   f->setName("f0");
   addFunction(f, AbstractTransformAxis::Rotational, AbstractTransformAxis::Rotational);

   const BodySet* bodySet = _engine->getBodySet();
   for (int i = 0; i < bodySet->getSize(); i++) {
      if (bodySet->get(i)->isA("Body")) {
         const Body* sb = (Body*)bodySet->get(i);
         convertBody(*sb, _engine->getMarkerSet());
      }
   }
}

//_____________________________________________________________________________
/**
 * Write a SIMM joint file.
 *
 * @param aFileName Name of joint file to write.
 * @return Whether or not file writing was successful.
 */
bool SimbodySimmModel::writeJointFile(const string& aFileName) const
{
	int i;
   ofstream out;
	int functionIndex = 1;

   out.open(aFileName.c_str());
   out.setf(ios::fixed);
   out.precision(6);

   if (!out.good())
   {
      cout << "Unable to open output file " << aFileName << endl;
      return false;
   }

   out << "/**********************************************************/\n";
   out << "/*            Joint file created by OpenSim               */\n";
   if (_engine->getModel()->getInputFileName() != "")
      out << "/* name of original model file: " << _engine->getModel()->getInputFileName() << " */\n";
   out << "/**********************************************************/\n";
   out << "\nname " << _engine->getModel()->getName() << endl;
   out << "\n";
#if 0
   if (!mBonePath.empty())
      out << "bone_path " << mBonePath << endl;
   if (!mMuscleFilename.empty())
      out << "\nmuscle_file " << mMuscleFilename << endl;
   if (!mMotionFilename.empty())
      out << "motion_file " << mMotionFilename << endl;
#endif

	if (_engine->getModel()->getLengthUnits().getType() != Units::simmUnknownUnits)
		out << "length_units " << _engine->getModel()->getLengthUnits().getLabel() << endl;
	if (_engine->getModel()->getForceUnits().getType() != Units::simmUnknownUnits)
		out << "force_units " << _engine->getModel()->getForceUnits().getLabel() << endl;

#if 0
   if (mMarkerRadius != 0.01)
      out << "marker_radius " << mMarkerRadius << endl;
   if (mMVGear != 0.08)
      out << "MV_gear " << mMVGear << endl;
   if (mMarkerVisibility)
      out << "marker_visibility on\n";
   else
      out << "marker_visibility off\n";
   smOptions options;
   this->getOptions(options);
   out << "solver_accuracy " << options.accuracy << endl;
   out << "solver_method " << getSolverMethodString(options.method) << endl;
   if (options.orientBodyToFrame == smYes)
      out << "solver_orient_body yes" << endl;
   else
      out << "solver_orient_body no" << endl;
   if (options.jointLimitsOn == smYes)
      out << "solver_joint_limits yes" << endl;
   else
      out << "solver_joint_limits no" << endl;
   out << "solver_max_iterations " << options.maxIterations << endl;
#endif

	SimTK::Vec3 gravity;
	_engine->getModel()->getGravity(gravity);
	const string& gravityLabel = getGravityLabel(gravity);

	out << "gravity " << gravityLabel << endl;

   out << "\n/****************************************************/\n";
   out << "/*                     SEGMENTS                     */\n";
   out << "/****************************************************/\n";
   for (i = 0; i < _simmBody.getSize(); i++)
      _simmBody[i]->write(out);

   out << "\n/****************************************************/\n";
   out << "/*                      JOINTS                      */\n";
   out << "/****************************************************/\n";
   for (i = 0; i < _simmJoint.getSize(); i++)
      _simmJoint[i]->write(out);

   out << "\n/****************************************************/\n";
   out << "/*                     GENCOORDS                    */\n";
   out << "/****************************************************/\n";
   for (i = 0; i < _simmGencoord.getSize(); i++)
      _simmGencoord[i]->write(out);

   out << "\n/****************************************************/\n";
   out << "/*                    WRAP OBJECTS                  */\n";
   out << "/****************************************************/\n";
   const BodySet* bodySet = _engine->getBodySet();
   for (i = 0; i < bodySet->getSize(); i++)
      writeWrapObjects(*bodySet->get(i), out);

   out << "\n/****************************************************/\n";
   out << "/*                     FUNCTIONS                    */\n";
   out << "/****************************************************/\n";
   for (i = 0; i < _simmFunction.getSize(); i++)
      _simmFunction[i]->write(out);
   
   out << "\n/****************************************************/\n";
   out << "/*                     MATERIALS                    */\n";
   out << "/****************************************************/\n";
   out << "beginmaterial mat1\nambient 0.3 0.3 0.9\ndiffuse 0.3 0.3 0.9\n";
   out << "specular 1.0 1.0 1.0\nendmaterial\n\n";

   out << "beginmaterial mat2\nambient 0.3 0.3 0.3\ndiffuse 0.3 0.3 0.3";
   out << "\nspecular 0.3 0.3 0.3\nendmaterial\n\n";

   out << "beginmaterial my_bone\nambient 0.65 0.65 0.65\nspecular 0.7 0.55 0.4";
   out << "\ndiffuse 0.55 0.4 0.35\nshininess 10\nendmaterial\n\n";

   out << "beginmaterial red\nambient 0.9 0.1 0.1\nspecular 0.7 0.2 0.2";
   out << "\ndiffuse 0.2 0.2 0.2\nshininess 10\nendmaterial\n\n";

   out << "beginmaterial blue\nambient 0.1 0.1 0.9\nspecular 0.2 0.2 0.7";
   out << "\ndiffuse 0.2 0.2 0.2\nshininess 10\nendmaterial\n\n";

   out << "\n/****************************************************/\n";
   out << "/*                     WORLD OBJECTS                */\n";
   out << "/****************************************************/\n";

	out << "beginworldobject floor\n";
   if (gravityLabel == "+X" || gravityLabel == "-X")
      out << "filename floor_yz_plane.asc\n";
   else if (gravityLabel == "+Y" || gravityLabel == "-Y")
      out << "filename floor_xz_plane.asc\n";
   else if (gravityLabel == "+Z" || gravityLabel == "-Z")
      out << "filename floor_xy_plane.asc\n";
   else
      out << "filename floor1.asc\n";

   out << "origin 0.0 0.0 0.0" << endl;
   out << "\nmaterial mat2\n";
	/* The floor bone file is in meters, so scale it to fit this model. */
	double floorScale = 1.0 / _engine->getModel()->getLengthUnits().convertTo(Units::simmMeters);
	out << "scale " << floorScale << " " << floorScale * 2.0 << " " << floorScale * 4.0 << endl;
   out << " endworldobject\n\n";

   out << "\n/****************************************************/\n";
   out << "/*                    MOTION OBJECTS                */\n";
   out << "/****************************************************/\n";

	/* The default ball object in SIMM is in meters, so scale it to fit this model. */
	out << "beginmotionobject ball\n";
	double scale = 0.25 / _engine->getModel()->getLengthUnits().convertTo(Units::simmMeters);
	out << "material blue" << endl;
	out << "scale " << scale << " " << scale << " " << scale << endl;
   out << "endmotionobject\n\n";

   out.close();
	//cout << "Wrote SIMM joint file " << aFileName << " from model " << _engine->getModel()->getName() << endl;

	return true;
}

//_____________________________________________________________________________
/**
 * Make a SIMM-compatible text label for the gravity vector.
 *
 * @param aGravity The gravity vector
 * @return Reference to the text label
 */
const string& SimbodySimmModel::getGravityLabel(const SimTK::Vec3& aGravity) const
{
	static string gravityLabels[] = {"-X","+X","-Y","+Y","-Z","+Z",""};

	if (aGravity[0] <= -9.8)
		return gravityLabels[0];
	if (aGravity[0] >= 9.8)
		return gravityLabels[1];
	if (aGravity[1] <= -9.8)
		return gravityLabels[2];
	if (aGravity[1] >= 9.8)
		return gravityLabels[3];
	if (aGravity[2] <= -9.8)
		return gravityLabels[4];
	if (aGravity[2] >= 9.8)
		return gravityLabels[5];

	return gravityLabels[6];
}

//_____________________________________________________________________________
/**
 * Determine if a coordinate is dependent on another.
 *
 * @param aCoordinate The coordinate to check for dependency.
 * @param rIndependentCoordinate The independent coordinate, if any, that the coordinate depends on.
 * @return The function controlling the dependency.
 */
OpenSim::Function* SimbodySimmModel::isDependent(const AbstractCoordinate* aCoordinate,
                                                 const AbstractCoordinate** rIndependentCoordinate) const
{
   for (int i=0; i<_engine->getConstraintSet()->getSize(); i++) {
      if (_engine->getConstraintSet()->get(i)->isA("CoordinateCouplerConstraint")) {
         const CoordinateCouplerConstraint* ccc = (CoordinateCouplerConstraint*)_engine->getConstraintSet()->get(i);
         if (ccc->getDependentCoordinateName() == aCoordinate->getName()) {
            string foo = ccc->getIndependentCoordinateNames().get(0);
            *rIndependentCoordinate = _engine->getCoordinateSet()->get(ccc->getIndependentCoordinateNames().get(0));
            return ccc->getFunction();
         }
      }
   }

   *rIndependentCoordinate = NULL;
   return NULL;
}

//_____________________________________________________________________________
/**
 * Convert a Simbody Body into one or more SIMM bodies and joints.
 *
 * @param aBody The body to convert.
 * @param aMarkerSet The marker set for the model.
 */
void SimbodySimmModel::convertBody(const OpenSim::Body& aBody, const MarkerSet* aMarkerSet)
{
   addBody(aBody);

   const Joint* joint = aBody.getJoint();
   if (joint == NULL)
      return;

   string parentName;
   string childName;
   addExtraJoints(*joint, parentName, childName);

   SimbodySimmJoint* ssj = new SimbodySimmJoint(joint->getName(), parentName, childName);

   if (joint->isA("CustomJoint")) {
      const CustomJoint* cj = (CustomJoint*)(joint);

      // Create the primary joint
      TransformAxisSet* dofs = cj->getTransformAxisSet();
      for (int i=0; i<dofs->getSize(); i++) {
         AbstractTransformAxis* ta = dofs->get(i);
         const Function* constraintFunc = ta->getFunction();
         const AbstractCoordinate* coord = ta->getCoordinate();
         const AbstractCoordinate* independentCoord = NULL;
         if (constraintFunc) { // dof is constrained to a coordinate in this joint
            ssj->addFunctionDof(*ta, coord->getName(), addFunction(constraintFunc, coord->getMotionType(), ta->getMotionType()));
         } else if ((constraintFunc = isDependent(coord, &independentCoord)) != NULL) { // dof is constrained to a coordinate in another joint
            ssj->addFunctionDof(*ta, independentCoord->getName(), addFunction(constraintFunc, independentCoord->getMotionType(), ta->getMotionType()));
         } else { // dof is unconstrained
            ssj->addFunctionDof(*ta, coord->getName(), 0);
         }
         SimTK::Vec3 axis;
         ta->getAxis(axis);
      }

      // Create gencoords from the coordinates
      CoordinateSet* coordinates = cj->getCoordinateSet();
      for (int i=0; i<coordinates->getSize(); i++) {
         AbstractCoordinate* coord = coordinates->get(i);
         const AbstractCoordinate* independentCoord = NULL;
         if (isDependent(coord, &independentCoord) == NULL) {
            addGencoord(coord);
         }
      }
      ssj->finalize();
      _simmJoint.append(ssj);
   }
}

//_____________________________________________________________________________
/**
 * Check to see if a translation and XYZ Euler rotation are all zeros.
 *
 * @param aLocation The translation vector.
 * @param aOrientation The XYZ Euler rotation.
 * @return Whether or not a joint is needed to represent the translation and rotation.
 */
bool SimbodySimmModel::isJointNeeded(SimTK::Vec3& aLocation, SimTK::Vec3& aOrientation)
{
   double sum = aLocation.scalarNormSqr();
   if (NOT_EQUAL_WITHIN_ERROR(sum, 0.0))
      return true;

   sum = aOrientation.scalarNormSqr();
   if (NOT_EQUAL_WITHIN_ERROR(sum, 0.0))
      return true;

   return false;
}

//_____________________________________________________________________________
/**
 * Make a joint of all constants to represent a translation and XYZ Euler rotation.
 *
 * @param aName The name of the joint.
 * @param aParentName The name of the joint's parent body.
 * @param aChildName The name of the joint's child body.
 * @param aLocation The translation.
 * @param aOrientation The XYZ Euler rotation.
 */
void SimbodySimmModel::makeSimmJoint(const string& aName, const string& aParentName, const string& aChildName,
                                     SimTK::Vec3& aLocation, SimTK::Vec3& aOrientation)
{
   SimbodySimmJoint* ssj = new SimbodySimmJoint(aName, aParentName, aChildName);
   ssj->addConstantDof("tx", NULL, aLocation[0]);
   ssj->addConstantDof("ty", NULL, aLocation[1]);
   ssj->addConstantDof("tz", NULL, aLocation[2]);
   ssj->addConstantDof("r1", NULL, aOrientation[0]);
   ssj->addConstantDof("r2", NULL, aOrientation[1]);
   ssj->addConstantDof("r3", NULL, aOrientation[2]);
   ssj->finalize();
   _simmJoint.append(ssj);
}

//_____________________________________________________________________________
/**
 * Add extra joints to the SIMM model to represent the joint location/orientation
 * in the parent body and in the child body.
 *
 * @param aJoint The Simbody joint.
 * @param rParentName The name of the parent body for the "real" joint.
 * @param rChildName The name of the child body for the "real" joint.
 */
void SimbodySimmModel::addExtraJoints(const OpenSim::Joint& aJoint, string& rParentName, string& rChildName)
{
   SimTK::Vec3 location;
   SimTK::Vec3 orientation;

   aJoint.getLocationInParent(location);
   aJoint.getOrientationInParent(orientation);
   bool needJoint = isJointNeeded(location, orientation);

   if (needJoint) {
      string bodyName = aJoint.getBody()->getName() + "_pjc";
      SimbodySimmBody* b = new SimbodySimmBody(NULL, bodyName);
      _simmBody.append(b);
      makeSimmJoint(aJoint.getName() + "_pjc", aJoint.getParentName(), bodyName, location, orientation);
      rParentName = bodyName;
   } else {
      rParentName = aJoint.getParentName();
   }

   aJoint.getLocation(location);
   aJoint.getOrientation(orientation);
   needJoint = isJointNeeded(location, orientation);
   if (needJoint) {
      string bodyName = aJoint.getBody()->getName() + "_jcc";
      SimbodySimmBody* b = new SimbodySimmBody(NULL, bodyName);
      _simmBody.append(b);
      // TODO: does this joint need to be reversed?
      makeSimmJoint(aJoint.getName() + "_jcc", bodyName, aJoint.getBody()->getName(), location, orientation);
      rChildName = bodyName;
   } else {
      rChildName = aJoint.getBody()->getName();
   }
}

//_____________________________________________________________________________
/**
 * Add a body to the SIMM model.
 *
 * @param aBody The Simbody body that the SimbodySimmBody is based on.
 */
void SimbodySimmModel::addBody(const OpenSim::Body& aBody)
{
   SimbodySimmBody* b = new SimbodySimmBody(&aBody, aBody.getName());
   _simmBody.append(b);
}

//_____________________________________________________________________________
/**
 * Add a gencoord to the SIMM model.
 *
 * @param aCoordinate The Simbody coordinate that the SimbodySimmGencoord is based on.
 */
void SimbodySimmModel::addGencoord(const AbstractCoordinate* aCoordinate)
{
	int restraintFuncNum = addFunction(aCoordinate->getRestraintFunction(),
		aCoordinate->getMotionType(), AbstractTransformAxis::Translational);
	int minRestraintFuncNum = addFunction(aCoordinate->getMinRestraintFunction(),
		aCoordinate->getMotionType(),	AbstractTransformAxis::Translational);
	int maxRestraintFuncNum = addFunction(aCoordinate->getMaxRestraintFunction(),
		aCoordinate->getMotionType(),	AbstractTransformAxis::Translational);
   SimbodySimmGencoord* g = new SimbodySimmGencoord(aCoordinate, restraintFuncNum,
		minRestraintFuncNum, maxRestraintFuncNum);
   _simmGencoord.append(g);
}

//_____________________________________________________________________________
/**
 * Add a function to the SimmModel and assign it a unique user number.
 *
 * TODO: check to make sure the function is a NaturalCubicSpline, currently the only
 * kind supported by SIMM (or 2-point linear or 3-point quadratic, which are implemented
 * in SIMM as special cases of natural cubics).
 *
 * @param aFunction The function to add.
 * @param aXType The type (translational, rotational) of the X coordinate of the function.
 * @param aYType The type (translational, rotational) of the Y coordinate of the function.
 * @return A unique user number for use in writing a SIMM joint file.
 */
int SimbodySimmModel::addFunction(const OpenSim::Function* aFunction, AbstractTransformAxis::MotionType aXType,
                                  AbstractTransformAxis::MotionType aYType)
{
	if (aFunction == NULL)
		return -1;

   // First see if there is already a function in _simmFunction that has the same
   // name and the same XY points.
   for (int i=0; i<_simmFunction.getSize(); i++) {
      const Function* f = _simmFunction.get(i)->getFunction();
      if (f->getName() == aFunction->getName() &&
         f->getNumberOfPoints() == aFunction->getNumberOfPoints() &&
         _simmFunction.get(i)->getXType() == aXType &&
         _simmFunction.get(i)->getYType() == aYType)
      {
         int j;
         for (j=0; j<f->getNumberOfPoints(); j++) {
            if (NOT_EQUAL_WITHIN_ERROR(f->getX(j), aFunction->getX(j)) ||
               NOT_EQUAL_WITHIN_ERROR(f->getY(j), aFunction->getY(j)))
               break;
         }
         if (j == f->getNumberOfPoints())
            return _simmFunction.get(i)->getUserNumber();
      }
   }

   // read user number from name
   int userNumber;
   string name = aFunction->getName();
   if (name.size() > 1 && name[0] == 'f') {
      userNumber = atoi(&name[1]);
   } else {
      userNumber = _simmFunction.getSize();
   }

   // Now make sure the user number is unique
   for (int i=0; i<_simmFunction.getSize(); i++) {
      if (userNumber == _simmFunction.get(i)->getUserNumber()) {
         userNumber = ++_maxFunctionUserNumber;
         break;
      }
   }

   if (userNumber > _maxFunctionUserNumber)
      _maxFunctionUserNumber = userNumber;

   SimbodySimmFunction* simmFunction = new SimbodySimmFunction(aFunction, userNumber, aXType, aYType);
   _simmFunction.append(simmFunction);

   return userNumber;
}

//_____________________________________________________________________________
/**
 * Write a body's wrap objects to a SIMM joint file.
 *
 * @param aBody reference to the body to write.
 * @param aStream the stream (file) to write to.
 */
void SimbodySimmModel::writeWrapObjects(AbstractBody& aBody, ofstream& aStream) const
{
	int i;
	WrapObjectSet& wrapObjects = aBody.getWrapObjectSet();

	for (i = 0; i < wrapObjects.getSize(); i++) {
		AbstractWrapObject* wo = wrapObjects.get(i);
		aStream << "beginwrapobject " << wo->getName() << endl;
		aStream << "wraptype " << wo->getWrapTypeName() << endl;
		aStream << "segment " << aBody.getName() << endl;
		aStream << wo->getDimensionsString() << endl;
		if (!wo->getQuadrantNameUseDefault())
			aStream << "quadrant " << wo->getQuadrantName() << endl;
		if (!wo->getActiveUseDefault())
			aStream << "active " << (wo->getActive() ? "yes" : "no") << endl;
		aStream << "translation " << wo->getTranslation()[0] << " " <<
			wo->getTranslation()[1] << " " << wo->getTranslation()[2] << endl;
		aStream << "xyz_body_rotation " << wo->getXYZBodyRotation()[0] * SimTK_RADIAN_TO_DEGREE <<
			" " << wo->getXYZBodyRotation()[1] * SimTK_RADIAN_TO_DEGREE <<
			" " << wo->getXYZBodyRotation()[2] * SimTK_RADIAN_TO_DEGREE << endl;
		aStream << "endwrapobject" << endl << endl;
	}
}
