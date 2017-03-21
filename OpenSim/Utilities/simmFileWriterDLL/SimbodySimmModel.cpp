// SimbodySimmModel.cpp
// Authors: Peter Loan
/*
 * Copyright (c) 2005-2017 Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
*   1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
*   2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
*   3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
*   4. Credits to developers may not be removed from executables
*     created from modifications of the source.
*   5. Modifications of source code must retain the above copyright notice, this list of
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
#include <fstream>
#include <iostream>
#include <string>
#include <math.h>
#include <float.h>
#include <time.h>

#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/EllipsoidJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/TransformAxis.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
#include <OpenSim/Simulation/Model/MovingPathPoint.h>
#include <OpenSim/Simulation/Wrap/WrapEllipsoid.h>
#include <OpenSim/Actuators/Thelen2003Muscle_Deprecated.h>
#include <OpenSim/Actuators/Schutte1993Muscle_Deprecated.h>
#include <OpenSim/Actuators/Delp1990Muscle_Deprecated.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Common/XYFunctionInterface.h>

#define ROUNDOFF_ERROR 0.000000001
#define DABS(a) ((a)>(double)0.0?(a):(-(a)))
#define EQUAL_WITHIN_ERROR(a,b) (DABS(((a)-(b))) <= ROUNDOFF_ERROR)
#define NOT_EQUAL_WITHIN_ERROR(a,b) (DABS(((a)-(b))) > ROUNDOFF_ERROR)
static const double defaultAxes[][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

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
 * Constructor from an OpenSim model.
 */
SimbodySimmModel::SimbodySimmModel(const Model* aModel)
{
    setNull();
    connectSimbodySimmModel(aModel);
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
SimbodySimmModel::SimbodySimmModel(const SimbodySimmModel& source)
:   SimbodySimmModel::Super(source)
{
    setNull();
    copyData(source);
}

//_____________________________________________________________________________
/**
 * Copy assignment.
 */
SimbodySimmModel& SimbodySimmModel::operator=(const SimbodySimmModel& source) {
    if (&source != this) {
        Super::operator=(source);
        setNull();
        copyData(source);
    }
    return *this;
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
    _model = aModel._model;
}

//_____________________________________________________________________________
/**
 * Set NULL values for all the variable members of this class.
 */
void SimbodySimmModel::setNull()
{
    _model = NULL;
    _maxFunctionUserNumber = 0;
    _uniqueJointNumber = 0;
}

//_____________________________________________________________________________
/**
 * Setup the SimbodySimmModel with information from a SimbodyEngine.
 *
 * @param aEngine The engine to make this SimbodySimmModel from.
 */
void SimbodySimmModel::connectSimbodySimmModel(const Model* aModel)
{
   _model = aModel;

   // Make a simple linear function for unconstrained coordinates
   SimmSpline* f = new SimmSpline();
   f->addPoint(-2.0*SimTK::Pi, -2.0*SimTK::Pi);
   f->addPoint(2.0*SimTK::Pi, 2.0*SimTK::Pi);
   f->setName("f0");
   addJointFunction(f, Coordinate::Rotational, Coordinate::Rotational);

   const BodySet& bodySet = _model->getBodySet();
   for (int i = 0; i < bodySet.getSize(); i++) {
      if (bodySet.get(i).getConcreteClassName()=="Body") {
         const Body& sb = bodySet.get(i);
         convertBody(sb);
      }
   }

   const JointSet& jointSet = _model->getJointSet();
   for (int i = 0; i < jointSet.getSize(); i++) {
           const Joint& sj = jointSet.get(i);
           convertJoint(sj);
   }
    // Create gencoords from the coordinates
    const CoordinateSet& coordinates = _model->getCoordinateSet();
    for (int i=0; i<coordinates.getSize(); i++) {
        Coordinate& coord = coordinates.get(i);
        const Coordinate* independentCoord = NULL;
        if (isDependent(&coord, &independentCoord) == NULL) {
            addGencoord(&coord);
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
   //int functionIndex = 1;

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
   if (_model->getInputFileName() != "")
      out << "/* name of original model file: " << _model->getInputFileName() << " */\n";
   out << "/**********************************************************/\n";
   out << "\nname " << _model->getName() << endl;
   out << "\n";
#if 0
   if (!mBonePath.empty())
      out << "bone_path " << mBonePath << endl;
   if (!mMuscleFilename.empty())
      out << "\nmuscle_file " << mMuscleFilename << endl;
   if (!mMotionFilename.empty())
      out << "motion_file " << mMotionFilename << endl;
#endif

    if (_model->getLengthUnits().getType() != Units::UnknownUnits)
        out << "length_units " << _model->getLengthUnits().getLabel() << endl;
    if (_model->getForceUnits().getType() != Units::UnknownUnits)
        out << "force_units " << _model->getForceUnits().getLabel() << endl;

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

    SimTK::Vec3 gravity=_model->getGravity();
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
   const BodySet& bodySet = _model->getBodySet();
   for (i = 0; i < bodySet.getSize(); i++)
      writeWrapObjects(bodySet.get(i), out);

   out << "\n/****************************************************/\n";
   out << "/*                     FUNCTIONS                    */\n";
   out << "/****************************************************/\n";
   for (i = 0; i < _simmJointFunction.getSize(); i++)
      _simmJointFunction[i]->write(out);

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
    double floorScale = 1.0 / _model->getLengthUnits().convertTo(Units::Meters);
    out << "scale " << floorScale << " " << floorScale * 2.0 << " " << floorScale * 4.0 << endl;
   out << " endworldobject\n\n";

   out << "\n/****************************************************/\n";
   out << "/*                    MOTION OBJECTS                */\n";
   out << "/****************************************************/\n";

    /* The default ball object in SIMM is in meters, so scale it to fit this model. */
    out << "beginmotionobject ball\n";
    double scale = 0.25 / _model->getLengthUnits().convertTo(Units::Meters);
    out << "material blue" << endl;
    out << "scale " << scale << " " << scale << " " << scale << endl;
   out << "endmotionobject\n\n";

   out.close();
    //cout << "Wrote SIMM joint file " << aFileName << " from model " << _model->getName() << endl;

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
const OpenSim::Function* SimbodySimmModel::isDependent(const Coordinate* aCoordinate,
                                                 const Coordinate** rIndependentCoordinate) const
{
   for (int i=0; i<_model->getConstraintSet().getSize(); i++) {
      if (_model->getConstraintSet().get(i).getConcreteClassName()==("CoordinateCouplerConstraint")) {
         const CoordinateCouplerConstraint& ccc = (CoordinateCouplerConstraint&)_model->getConstraintSet().get(i);
         if (ccc.getDependentCoordinateName() == aCoordinate->getName()) {
            string foo = ccc.getIndependentCoordinateNames().get(0);
            *rIndependentCoordinate = &_model->getCoordinateSet().get(ccc.getIndependentCoordinateNames().get(0));
            const OpenSim::Function* ret = &ccc.getFunction();
            return ret;
         }
      }
   }

   *rIndependentCoordinate = NULL;
   return NULL;
}

//_____________________________________________________________________________
/**
 * Convert a Simbody Body into one or more SIMM bodies and joints. In the general
 * case, the Simbody joint has a non-zero position and orientation in the parent
 * body, a non-zero position and orientation in the child body, and up to six
 * degrees of freedom. It is not always possible to convert this joint into a
 * single SIMM joint. In SIMM, the position and orientation of a joint in the
 * child body is always zero. Thus, the Simbody joint is converted using this
 * procedure:
 *   1. If the position and orientation in the child body is non-zero, an
 *      auxiliary SIMM joint is created just to hold this transform.
 *   2. If the position and orientation in the parent body plus the joint's
 *      degrees of freedom can be put into a single SIMM joint, it is done so.
 *      If not, an auxiliary joint is created just to hold the transform for
 *      the position and orientation in the parent body.
 *
 * @param aBody The body to convert.
 */
void SimbodySimmModel::convertBody(const OpenSim::Body& aBody)
{
    addBody(aBody);
}
/**
 * Convert an OpenSim Joint into the equivalent SIMM joint, only subset of joint types are supported
 * 
*/
void SimbodySimmModel::convertJoint(const Joint& joint)
{

   string parentName;
   string childName;
    bool parentJointAdded = addExtraJoints(joint, parentName, childName);

   SimbodySimmJoint* ssj = new SimbodySimmJoint(joint.getName(), parentName, childName);

    // If parentJointAdded is false, that means the position and orientation in the
    // parent can be merged with the primary joint. So begin making the joint by
    // adding the non-zero components to the SimbodySimmJoint.
    if (parentJointAdded == false) {
        int rotationsSoFar = 0;
        SimTK::Vec3 location(0);
        SimTK::Vec3 orientation(0);
        const PhysicalOffsetFrame* offset =
            dynamic_cast<const PhysicalOffsetFrame*>(&joint.getParentFrame());
        if (offset) {
            location = offset->get_translation();
            orientation = offset->get_orientation();
        }


        if (NOT_EQUAL_WITHIN_ERROR(location[0], 0.0))
            ssj->addConstantDof("tx", NULL, location[0]);
        if (NOT_EQUAL_WITHIN_ERROR(location[1], 0.0))
            ssj->addConstantDof("ty", NULL, location[1]);
        if (NOT_EQUAL_WITHIN_ERROR(location[2], 0.0))
            ssj->addConstantDof("tz", NULL, location[2]);
        if (NOT_EQUAL_WITHIN_ERROR(orientation[0], 0.0))
            ssj->addConstantDof(_rotationNames[rotationsSoFar++], defaultAxes[0], orientation[0] * 180.0 / SimTK::Pi);
        if (NOT_EQUAL_WITHIN_ERROR(orientation[1], 0.0))
            ssj->addConstantDof(_rotationNames[rotationsSoFar++], defaultAxes[1], orientation[1] * 180.0 / SimTK::Pi);
        if (NOT_EQUAL_WITHIN_ERROR(orientation[2], 0.0))
            ssj->addConstantDof(_rotationNames[rotationsSoFar++], defaultAxes[2], orientation[2] * 180.0 / SimTK::Pi);
    }

    if (joint.getConcreteClassName()==("WeldJoint")) {
        // Nothing to do.
    } else if (joint.getConcreteClassName()==("PinJoint")) {
        int index = 0;
        string coordName = joint.get_coordinates(index).getName();
        SimTK::Vec3 axis(0.0, 0.0, 1.0); // Pin joints always rotate about the Z axis.
        ssj->addFunctionDof(axis, coordName, 0, Coordinate::Rotational);
    } else if (joint.getConcreteClassName()==("SliderJoint")) {
        int index = 0;
        string coordName = joint.get_coordinates(index).getName();
        SimTK::Vec3 axis(1.0, 0.0, 0.0); // Slider joints always translate along the X axis.
        ssj->addFunctionDof(axis, coordName, 0, Coordinate::Translational);
    } else if (joint.getConcreteClassName()==("EllipsoidJoint")) {
        // NOTE: Ellipsoid joints cannot be converted into SIMM joints.
    } else if (joint.getConcreteClassName()==("FreeJoint")) {
        SimTK::Vec3 xaxis(1.0, 0.0, 0.0);
        SimTK::Vec3 yaxis(0.0, 1.0, 0.0);
        SimTK::Vec3 zaxis(0.0, 0.0, 1.0);
        int index = 0;
        ssj->addFunctionDof(xaxis, joint.get_coordinates(index++).getName(), 0, Coordinate::Translational);
        ssj->addFunctionDof(yaxis, joint.get_coordinates(index++).getName(), 0, Coordinate::Translational);
        ssj->addFunctionDof(zaxis, joint.get_coordinates(index++).getName(), 0, Coordinate::Translational);
        ssj->addFunctionDof(xaxis, joint.get_coordinates(index++).getName(), 0, Coordinate::Rotational);
        ssj->addFunctionDof(yaxis, joint.get_coordinates(index++).getName(), 0, Coordinate::Rotational);
        ssj->addFunctionDof(zaxis, joint.get_coordinates(index).getName(), 0, Coordinate::Rotational);
    } else if (joint.getConcreteClassName()==("CustomJoint")) {
        const CustomJoint* cj = (CustomJoint*)(&joint);

        // Add the joint's transform axes to the SimbodySimmJoint.
        const SpatialTransform& dofs = cj->getSpatialTransform();
        // Custom joints have the rotational DOFs specified first, but the translations are applied first
        // so you want to process them first here.
        static int order[] = {3, 4, 5, 0, 1, 2};
        for (int i=0; i<6; i++) {
            const TransformAxis* ta = &dofs[order[i]];
            if (ta->getCoordinateNames().size() > 0) { // transform axis is unused if it has no coordinate names
                const Coordinate* coord = NULL;
                const Coordinate* independentCoord = NULL;
                const Function* constraintFunc = NULL;
                int ix = -1;
                Coordinate::MotionType motionType = (order[i]<3) ? Coordinate::Rotational : Coordinate::Translational;
                if (ta->getCoordinateNames().size() > 0)
                    ix = cj->getProperty_coordinates().findIndexForName(ta->getCoordinateNames()[0]);
                if (ix >= 0) {
                    coord = &cj->get_coordinates(ix);
                    constraintFunc = isDependent(coord, &independentCoord);
                }
                if (constraintFunc != NULL) {  // dof is constrained to a coordinate in another joint
                    ssj->addFunctionDof(ta->getAxis(), independentCoord->getName(), addJointFunction(constraintFunc, independentCoord->getMotionType(), motionType), motionType);
                } else {
                    if (ta->hasFunction())
                        constraintFunc = &ta->getFunction();
                    if (constraintFunc) // dof is constrained to a coordinate in this joint
                        ssj->addFunctionDof(ta->getAxis(), coord->getName(), addJointFunction(constraintFunc, coord->getMotionType(), motionType), motionType);
                    else // dof is unconstrained
                        ssj->addFunctionDof(ta->getAxis(), coord->getName(), 0, motionType);
                }
            }
        }
    }

    ssj->finalize();
    addSimmJoint(ssj);
}

//_____________________________________________________________________________
/**
 * Check to see if a location (in child) and orientation (in child) are all zeros.
 * If they are not, then a separate SIMM joint is needed to hold this transform.
 *
 * @param aJoint The Simbody joint to check.
 * @return Whether or not a separate SIMM joint is needed to represent the location
 *         and orientation in child.
 */
bool SimbodySimmModel::isChildJointNeeded(const OpenSim::Joint& aJoint)
{
    SimTK::Vec3 location(0);
    SimTK::Vec3 orientation(0);
    const PhysicalOffsetFrame* offset =
        dynamic_cast<const PhysicalOffsetFrame*>(&aJoint.getChildFrame());
    if (offset) {
        location = offset->get_translation();
        orientation = offset->get_orientation();
    }
    else {
        return false;
    }

    double sum = location.scalarNormSqr();
   if (NOT_EQUAL_WITHIN_ERROR(sum, 0.0))
      return true;

   sum = orientation.scalarNormSqr();
   if (NOT_EQUAL_WITHIN_ERROR(sum, 0.0))
      return true;

   return false;
}

//_____________________________________________________________________________
/**
 * Check to see if the location (in parent) and orientation (in parent) of a
 * Simbody joint can be merged with the joint's "real" DOFs and still be
 * represented with a single SIMM joint.
 *
 * @param aJoint The Simbody joint to check.
 * @return Whether or not a separate SIMM joint is needed to represent the location
 *         and orientation in parent.
 */
bool SimbodySimmModel::isParentJointNeeded(const OpenSim::Joint& aJoint)
{
    // If the joint has no "real" DOFs (e.g., WeldJoints), then the parent joint is not needed.
    if (aJoint.getConcreteClassName()=="WeldJoint")
        return false;

    SimTK::Vec3 location(0);
    SimTK::Vec3 orientation(0);

    const PhysicalOffsetFrame* offset =
        dynamic_cast<const PhysicalOffsetFrame*>(&aJoint.getParentFrame());
    if (offset) {
        location = offset->get_translation();
        orientation = offset->get_orientation();
    }

    bool translationsUsed[] = {false, false, false}, translationsDone = false;
    int numTranslations = 0, numRotations = 0;

    // First see which components are needed for the location and orientation.
    for (int i=0; i<3; i++) {
        if (NOT_EQUAL_WITHIN_ERROR(location[i], 0.0)) {
            translationsUsed[i] = true;
            numTranslations++;
        }
    }
    for (int i=0; i<3; i++) {
        if (NOT_EQUAL_WITHIN_ERROR(orientation[i], 0.0)) {
            numRotations++;
            if (numTranslations > 0)
                translationsDone = true;
        }
    }

    if (aJoint.getConcreteClassName()==("PinJoint")) {
        if (numRotations >= 3) // have already defined three rotations (can't add more)
            return true;
    } else if (aJoint.getConcreteClassName()==("SliderJoint")) {
        if (translationsUsed[0]) // slider joint translations are always along the X axis
            return true;
    } else if (aJoint.getConcreteClassName()==("EllipsoidJoint")) {
        return true; // NOTE: ellipsoid joints cannot be converted to SIMM joints
    } else if (aJoint.getConcreteClassName()==("FreeJoint")) {
        return true;
    } else if (aJoint.getConcreteClassName()==("CustomJoint")) {
        const CustomJoint* cj = (CustomJoint*)(&aJoint);
        const SpatialTransform& dofs = cj->getSpatialTransform();

        // Now see if the joint's "real" DOFs can be added.
        for (int i=0; i<6; i++) {
            const TransformAxis* ta = &dofs[i];
            if (i >= 3) {
                const auto& axis = ta->getAxis();
                for (int j=0; j<3; j++) {
                    if (EQUAL_WITHIN_ERROR(axis[j], 1.0)) {
                        if (ta->getCoordinateNames().size() > 0) { // transform axis is unused if it has no coordinate names
                            if (translationsUsed[i-3]) // this translation component already defined
                                return true;
                            if (translationsDone) // have already defined translations then rotation (can't add more translations)
                                return true;
                            translationsUsed[i-3] = true;
                            numTranslations++;
                        }
                        break;
                    }
                }
            } else {
                if (ta->getCoordinateNames().size() > 0) { // transform axis is unused if it has no coordinate names
                    if (numRotations >= 3) // have already defined three rotations (can't add more)
                        return true;
                    numRotations++;
                    if (numTranslations > 0)
                        translationsDone = true;
                }
            }
        }
    }

   return false;
}

bool SimbodySimmModel::jointArrayContains(const string& aName)
{
    for (int i=0; i<_simmJoint.getSize(); i++)
        if (_simmJoint.get(i)->getName() == aName)
            return true;

    return false;
}

void SimbodySimmModel::addSimmJoint(SimbodySimmJoint* joint)
{
   // Make sure the name is unique
   string uniqueName = joint->getName();
   while (jointArrayContains(uniqueName))
   {
       ostringstream num;
       num << _uniqueJointNumber++;
       uniqueName = joint->getName() + "_" + num.str();
   }

   joint->setName(uniqueName);
   _simmJoint.append(joint);
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
   ssj->addConstantDof("r1", defaultAxes[0], aOrientation[0] * 180.0 / SimTK::Pi);
   ssj->addConstantDof("r2", defaultAxes[1], aOrientation[1] * 180.0 / SimTK::Pi);
   ssj->addConstantDof("r3", defaultAxes[2], aOrientation[2] * 180.0 / SimTK::Pi);
   ssj->finalize();
   addSimmJoint(ssj);
}

//_____________________________________________________________________________
/**
 * Add extra joints to the SIMM model to represent the joint location/orientation
 * in the parent body and in the child body.
 *
 * @param aJoint The Simbody joint.
 * @param rParentName The name of the parent body for the "real" joint.
 * @param rChildName The name of the child body for the "real" joint.
 * @return Whether or not a 'parent' joint was added.
 */
bool SimbodySimmModel::addExtraJoints(const OpenSim::Joint& aJoint, string& rParentName, string& rChildName)
{
   SimTK::Vec3 location(0);
   SimTK::Vec3 orientation(0);
    bool parentJointAdded = false;

   if (isParentJointNeeded(aJoint)) {
       const PhysicalOffsetFrame* offset =
           dynamic_cast<const PhysicalOffsetFrame*>(&aJoint.getParentFrame());
       if (offset) {
           location = offset->get_translation();
           orientation = offset->get_orientation();
       }

      string bodyName = aJoint.getChildFrame().getName() + "_pjc";
      SimbodySimmBody* b = new SimbodySimmBody(NULL, bodyName);
      _simmBody.append(b);
      makeSimmJoint(aJoint.getName() + "_pjc", 
          aJoint.getParentFrame().getName(), bodyName, location, orientation);
      rParentName = bodyName;
        parentJointAdded = true;
   } else {
      rParentName = aJoint.getParentFrame().getName();
        parentJointAdded = false;
   }

   location = 0;
   orientation = 0;

   if (isChildJointNeeded(aJoint)) {
       const PhysicalOffsetFrame* offset =
           dynamic_cast<const PhysicalOffsetFrame*>(&aJoint.getChildFrame());
       if (offset) {
           location = offset->get_translation();
           orientation = offset->get_orientation();
       }
       string bodyName = aJoint.getChildFrame().getName() + "_jcc";
      SimbodySimmBody* b = new SimbodySimmBody(NULL, bodyName);
      _simmBody.append(b);
      // This joint is specified in the reverse direction.
      makeSimmJoint(aJoint.getName() + "_jcc",
          aJoint.getChildFrame().getName(), bodyName, location, orientation);
      rChildName = bodyName;
   } else {
       rChildName = aJoint.getChildFrame().getName();
   }

    return parentJointAdded;
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
void SimbodySimmModel::addGencoord(const Coordinate* aCoordinate)
{
   SimbodySimmGencoord* g = new SimbodySimmGencoord(aCoordinate);
   _simmGencoord.append(g);
}

//_____________________________________________________________________________
/**
 * Add a function to the SimmModel (_simmJointFunction) and assign it a unique user number.
 *
 * @param aFunction The function to add.
 * @param aXType The type (translational, rotational) of the X coordinate of the function.
 * @param aYType The type (translational, rotational) of the Y coordinate of the function.
 * @return A unique user number for use in writing a SIMM joint file.
 */
int SimbodySimmModel::addJointFunction(const OpenSim::Function* aFunction, Coordinate::MotionType aXType, Coordinate::MotionType aYType)
{
    if (aFunction == NULL)
        return -1;

    XYFunctionInterface xyFuncNew((OpenSim::Function*)aFunction); // removing const!

   // First see if there is already a function in _simmJointFunction that has the same
   // name and the same XY points.
   for (int i=0; i<_simmJointFunction.getSize(); i++) {
      const Function* f = _simmJointFunction.get(i)->getFunction();
        XYFunctionInterface xyFunc((OpenSim::Function*)f); // removing const!
      if (f->getName() == aFunction->getName() &&
         xyFunc.getNumberOfPoints() == xyFuncNew.getNumberOfPoints() &&
         _simmJointFunction.get(i)->getXType() == aXType &&
         _simmJointFunction.get(i)->getYType() == aYType)
      {
         int j;
         for (j=0; j<xyFunc.getNumberOfPoints(); j++) {
            if (NOT_EQUAL_WITHIN_ERROR(xyFunc.getX(j), xyFuncNew.getX(j)) ||
               NOT_EQUAL_WITHIN_ERROR(xyFunc.getY(j), xyFuncNew.getY(j)))
               break;
         }
         if (j == xyFunc.getNumberOfPoints())
            return _simmJointFunction.get(i)->getUserNumber();
      }
   }

    int userNumber = getUniqueFunctionUserNumber(aFunction);

   SimbodySimmFunction* simmFunction = new SimbodySimmFunction(aFunction, userNumber, aXType, aYType);
   _simmJointFunction.append(simmFunction);

   return userNumber;
}

//_____________________________________________________________________________
/**
 * Add a function to the SimmModel (_simmMuscleFunction) and assign it a unique user number.
 *
 * @param aFunction The function to add.
 * @param aXType The type (translational, rotational) of the X coordinate of the function.
 * @param aYType The type (translational, rotational) of the Y coordinate of the function.
 * @return A unique user number for use in writing a SIMM joint file.
 */
int SimbodySimmModel::addMuscleFunction(const OpenSim::Function* aFunction, Coordinate::MotionType aXType, Coordinate::MotionType aYType)
{
    if (aFunction == NULL)
        return -1;

    XYFunctionInterface xyFuncNew((OpenSim::Function*)aFunction); // removing const!

   // First see if there is already a function in _simmMuscleFunction that has the same
   // name and the same XY points.
   for (int i=0; i<_simmMuscleFunction.getSize(); i++) {
      const Function* f = _simmMuscleFunction.get(i)->getFunction();
        XYFunctionInterface xyFunc((OpenSim::Function*)f); // removing const!
      if (f->getName() == aFunction->getName() &&
         xyFunc.getNumberOfPoints() == xyFuncNew.getNumberOfPoints() &&
         _simmMuscleFunction.get(i)->getXType() == aXType &&
         _simmMuscleFunction.get(i)->getYType() == aYType)
      {
         int j;
         for (j=0; j<xyFunc.getNumberOfPoints(); j++) {
            if (NOT_EQUAL_WITHIN_ERROR(xyFunc.getX(j), xyFuncNew.getX(j)) ||
               NOT_EQUAL_WITHIN_ERROR(xyFunc.getY(j), xyFuncNew.getY(j)))
               break;
         }
         if (j == xyFunc.getNumberOfPoints())
            return _simmMuscleFunction.get(i)->getUserNumber();
      }
   }

    int userNumber = getUniqueFunctionUserNumber(aFunction);

   SimbodySimmFunction* simmFunction = new SimbodySimmFunction(aFunction, userNumber, aXType, aYType);
   _simmMuscleFunction.append(simmFunction);

   return userNumber;
}

//_____________________________________________________________________________
/**
 * Write a body's wrap objects to a SIMM joint file.
 *
 * @param aBody reference to the body to write.
 * @param aStream the stream (file) to write to.
 */
void SimbodySimmModel::writeWrapObjects(OpenSim::Body& aBody, ofstream& aStream) const
{
    int i;
    const WrapObjectSet& wrapObjects = aBody.getWrapObjectSet();

    for (i = 0; i < wrapObjects.getSize(); i++) {
        WrapObject& wo = wrapObjects.get(i);
        aStream << "beginwrapobject " << wo.getName() << endl;
        aStream << "wraptype " << wo.getWrapTypeName() << endl;
        aStream << "segment " << aBody.getName() << endl;
        aStream << wo.getDimensionsString() << endl;
        if (!wo.getQuadrantNameUseDefault())
            aStream << "quadrant " << wo.get_quadrant() << endl;
        if (!wo.getActiveUseDefault())
            aStream << "active " << (wo.get_active() ? "yes" : "no") << endl;
        aStream << "translation " << wo.get_translation()[0] << " " <<
            wo.get_translation()[1] << " " << wo.get_translation()[2] << endl;
        aStream << "xyz_body_rotation " << wo.get_xyz_body_rotation()[0] * SimTK_RADIAN_TO_DEGREE <<
            " " << wo.get_xyz_body_rotation()[1] * SimTK_RADIAN_TO_DEGREE <<
            " " << wo.get_xyz_body_rotation()[2] * SimTK_RADIAN_TO_DEGREE << endl;
        aStream << "endwrapobject" << endl << endl;
    }
}

int SimbodySimmModel::getUniqueFunctionUserNumber(const OpenSim::Function* aFunction)
{
    int userNumber = -1;

    if (aFunction) {
        // Try to read the user number from the function name.
        string name = aFunction->getName();
        if (name.size() > 1 && name[0] == 'f') {
            userNumber = atoi(&name[1]);

            // Make sure the user number is not already used by a joint function.
            for (int i=0; i<_simmJointFunction.getSize(); i++) {
                if (userNumber == _simmJointFunction.get(i)->getUserNumber()) {
                    userNumber = -1;
                    break;
                }
            }

            if (userNumber != -1) {
                // Make sure the user number is not already used by a muscle function.
                for (int i=0; i<_simmMuscleFunction.getSize(); i++) {
                    if (userNumber == _simmMuscleFunction.get(i)->getUserNumber()) {
                        userNumber = -1;
                        break;
                    }
                }
            }
        }
    }

    if (userNumber == -1)
        userNumber = _maxFunctionUserNumber + 1;

    if (userNumber > _maxFunctionUserNumber)
        _maxFunctionUserNumber = userNumber;

    return userNumber;
}

bool SimbodySimmModel::writeMuscleFile(const string& aFileName)
{
   ofstream out;

   out.open(aFileName.c_str());
   out.setf(ios::fixed);
   out.precision(12);

   if (!out.good())
   {
      cout << "Unable to open output muscle file " << aFileName << endl;
      return false;
   }

   out << "/**********************************************************/\n";
   out << "/*            Muscle file created by OpenSim              */\n";
   if (_model->getInputFileName() != "")
      out << "/* name of original model file: " << _model->getInputFileName() << " */\n";
   out << "/**********************************************************/\n";
   out << "\n";

    /* TODO: hack to support dynamic parameters in all currently defined muscle models. */
    out << "begindynamicparameters" << endl;
    out << "timescale" << endl;
    out << "mass" << endl;
    out << "damping" << endl;
    out << "activation1" << endl;
    out << "activation2" << endl;
    out << "activation_time_constant" << endl;
    out << "deactivation_time_constant" << endl;
    out << "Vmax" << endl;
    out << "Vmax0" << endl;
    out << "Af" << endl;
    out << "Flen" << endl;
    out << "FmaxTendonStrain" << endl;
    out << "FmaxMuscleStrain" << endl;
    out << "KshapeActive" << endl;
    out << "KshapePassive" << endl;
    out << "muscle_density" << endl;
    out << "max_isometric_stress" << endl;
    out << "enddynamicparameters" << endl << endl;

    /* The default muscle must be defined or the Pipeline code crashes. */
    out << "beginmuscle defaultmuscle" << endl;
    out << "endmuscle" << endl << endl;

    Array<SimbodySimmFunction*> simmMuscleFunction;
    const ForceSet& actuatorSet = _model->getForceSet();

    for (int i=0; i<actuatorSet.getSize(); i++) {
        Muscle* muscle = dynamic_cast<Muscle*>(&actuatorSet.get(i));
        if (muscle)
            writeMuscle(*muscle, actuatorSet, out);
    }

   out << "\n/****************************************************/\n";
   out << "/*                     FUNCTIONS                    */\n";
   out << "/****************************************************/\n";
   for (int i=0; i<_simmMuscleFunction.getSize(); i++)
      _simmMuscleFunction[i]->write(out);

   out.close();
    cout << "Wrote SIMM muscle file " << aFileName << " from model " << _model->getName() << endl;

    return true;
}

bool SimbodySimmModel::writeMuscle(Muscle& aMuscle, const ForceSet& aActuatorSet, ofstream& aStream)
{
    aStream << "beginmuscle " << aMuscle.getName() << endl;

    const PathPointSet& pts = aMuscle.getGeometryPath().getPathPointSet();

    // get a state for the purpose of probing path points
    const SimTK::State& s = aMuscle.getSystem().getDefaultState();

    aStream << "beginpoints" << endl;
    for (int i = 0; i < pts.getSize(); i++)
    {
        AbstractPathPoint& pt = pts.get(i);
        if (pt.getConcreteClassName()==("ConditionalPathPoint")) {
            ConditionalPathPoint* mvp = (ConditionalPathPoint*)(&pt);
            Vec3 attachment = mvp->getLocation(s);
            double range[]{ mvp->get_range(0), mvp->get_range(1) };
            aStream << attachment[0] << " " << attachment[1] << " " << attachment[2] << " segment " << mvp->getParentFrame().getName();
            
            if (mvp->hasCoordinate()) {
                const Coordinate& coord = mvp->getCoordinate();
                if (coord.getMotionType() == Coordinate::Rotational)
                    aStream << " ranges 1 " << coord.getName() << " (" << range[0] * SimTK_RADIAN_TO_DEGREE << ", " << range[1] * SimTK_RADIAN_TO_DEGREE << ")" << endl;
                else
                    aStream << " ranges 1 " << coord.getName() << " (" << range[0] << ", " << range[1] << ")" << endl;
            } else {
                aStream << " ranges 1 " 
                    << mvp->getSocket<Coordinate>("coordinate").getConnecteeName()
                    << " (0.0, 1.0)" << endl;
            }
        } else if (pt.getConcreteClassName()==("MovingPathPoint")) {
            MovingPathPoint* mpp = (MovingPathPoint*)(&pt);
            const Vec3 attachment(0);
            if (mpp->hasXCoordinate()) {
                aStream << "f" << addMuscleFunction(&mpp->get_x_location(), mpp->getXCoordinate().getMotionType(), Coordinate::Translational) << "(" << mpp->getXCoordinate().getName() << ") ";
            } else {
                aStream << attachment[0] << " ";
            }
            if (mpp->hasYCoordinate()) {
                aStream << "f" << addMuscleFunction(&mpp->get_y_location(), mpp->getYCoordinate().getMotionType(), Coordinate::Translational) << "(" << mpp->getYCoordinate().getName() << ") ";
            } else {
                aStream << attachment[1] << " ";
            }
            if (mpp->hasZCoordinate()) {
                aStream << "f" << addMuscleFunction(&mpp->get_z_location(), mpp->getZCoordinate().getMotionType(), Coordinate::Translational) << "(" << mpp->getZCoordinate().getName() << ")";
            } else {
                aStream << attachment[2];
            }
            aStream << " segment " << mpp->getParentFrame().getName() << endl;
        } else {
            Vec3 attachment = pt.getLocation(s);
            aStream << attachment[0] << " " << attachment[1] << " " << attachment[2] << " segment " << pt.getParentFrame().getName() << endl;
        }
    }
    aStream << "endpoints" << endl;

    Array<std::string> groupNames;
    aActuatorSet.getGroupNamesContaining(aMuscle.getName(),groupNames);
    if(groupNames.getSize()) {
        aStream << "begingroups" << endl;
        for(int i=0; i<groupNames.getSize(); i++)
            aStream << " " << groupNames[i];
        aStream << endl << "endgroups" << endl;
    }

    const PathWrapSet& wrapObjects = aMuscle.getGeometryPath().getWrapSet();
    for (int i=0; i<wrapObjects.getSize(); i++)
        aStream << "wrapobject " << wrapObjects.get(i).getWrapObjectName() << " " <<
        (dynamic_cast<WrapEllipsoid*>(&wrapObjects.get(i)) ? (wrapObjects.get(i).getMethodName()+" ") : "") <<
        "range " << wrapObjects.get(i).getStartPoint() << " " << wrapObjects.get(i).getEndPoint() << endl;

    if (dynamic_cast<Schutte1993Muscle_Deprecated*>(&aMuscle))
    {
        Schutte1993Muscle_Deprecated *szh = dynamic_cast<Schutte1993Muscle_Deprecated*>(&aMuscle);

        aStream << "max_force " << szh->getMaxIsometricForce() << endl;
        aStream << "optimal_fiber_length " << szh->getOptimalFiberLength() << endl;
        aStream << "tendon_slack_length " << szh->getTendonSlackLength() << endl;
        aStream << "pennation_angle " << szh->getPennationAngleAtOptimalFiberLength() * SimTK_RADIAN_TO_DEGREE << endl;
        aStream << "max_contraction_velocity " << szh->getMaxContractionVelocity() << endl;
        aStream << "timescale " << szh->getTimeScale() << endl;
        aStream << "muscle_model 4" << endl;

        aStream << "active_force_length_curve f" << addMuscleFunction(&szh->getActiveForceLengthCurve(), Coordinate::Translational, Coordinate::Translational) << endl;

        aStream << "passive_force_length_curve f" << addMuscleFunction(&szh->getPassiveForceLengthCurve(), Coordinate::Translational, Coordinate::Translational) << endl;

        aStream << "tendon_force_length_curve f" << addMuscleFunction(&szh->getTendonForceLengthCurve(), Coordinate::Translational, Coordinate::Translational) << endl;
    }
    else if (dynamic_cast<Thelen2003Muscle_Deprecated*>(&aMuscle))
    {
        Thelen2003Muscle_Deprecated *sdm = dynamic_cast<Thelen2003Muscle_Deprecated*>(&aMuscle);

        aStream << "max_force " << sdm->getMaxIsometricForce() << endl;
        aStream << "optimal_fiber_length " << sdm->getOptimalFiberLength() << endl;
        aStream << "tendon_slack_length " << sdm->getTendonSlackLength() << endl;
        aStream << "pennation_angle " << sdm->getPennationAngleAtOptimalFiberLength() * SimTK_RADIAN_TO_DEGREE << endl;
        aStream << "activation_time_constant " << sdm->getActivationTimeConstant() << endl;
        aStream << "deactivation_time_constant " << sdm->getDeactivationTimeConstant() << endl;
        aStream << "Vmax " << sdm->getVmax() << endl;
        aStream << "Vmax0 " << sdm->getVmax0() << endl;
        aStream << "FmaxTendonStrain " << sdm->getFmaxTendonStrain() << endl;
        aStream << "FmaxMuscleStrain " << sdm->getFmaxMuscleStrain() << endl;
        aStream << "KshapeActive " << sdm->getKshapeActive() << endl;
        aStream << "KshapePassive " << sdm->getKshapePassive() << endl;
        aStream << "damping " << sdm->getDamping() << endl;
        aStream << "Af " << sdm->getAf() << endl;
        aStream << "Flen " << sdm->getFlen() << endl;
        aStream << "muscle_model 9" << endl;
    }
    else if (dynamic_cast<Delp1990Muscle_Deprecated*>(&aMuscle))
    {
        Delp1990Muscle_Deprecated *szh = dynamic_cast<Delp1990Muscle_Deprecated*>(&aMuscle);

        aStream << "max_force " << szh->getMaxIsometricForce() << endl;
        aStream << "optimal_fiber_length " << szh->getOptimalFiberLength() << endl;
        aStream << "tendon_slack_length " << szh->getTendonSlackLength() << endl;
        aStream << "pennation_angle " << szh->getPennationAngleAtOptimalFiberLength() * SimTK_RADIAN_TO_DEGREE << endl;
        aStream << "max_contraction_velocity " << szh->getMaxContractionVelocity() << endl;
        aStream << "timescale " << szh->getTimeScale() << endl;
        aStream << "muscle_model 2" << endl;

        if (szh->getActiveForceLengthCurve())
            aStream << "active_force_length_curve f" << addMuscleFunction(szh->getActiveForceLengthCurve(), Coordinate::Translational, Coordinate::Translational) << endl;

        if (szh->getPassiveForceLengthCurve())
            aStream << "passive_force_length_curve f" << addMuscleFunction(szh->getPassiveForceLengthCurve(), Coordinate::Translational, Coordinate::Translational) << endl;

        if (szh->getTendonForceLengthCurve())
            aStream << "tendon_force_length_curve f" << addMuscleFunction(szh->getTendonForceLengthCurve(), Coordinate::Translational, Coordinate::Translational) << endl;

        if (szh->getForceVelocityCurve())
            aStream << "force_velocity_curve f" << addMuscleFunction(szh->getForceVelocityCurve(), Coordinate::Translational, Coordinate::Translational) << endl;
    }

    aStream << "endmuscle" << endl << endl;
    return true;
}
