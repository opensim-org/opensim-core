#ifndef _osimSimulation_h_
#define _osimSimulation_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  osimSimulation.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): OpenSim Team                                                     *
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

#include "Model/AnalysisSet.h"
#include "Model/Model.h"
#include "Model/ModelDisplayHints.h"
#include "Model/ModelVisualizer.h"
#include "Model/ForceSet.h"
#include "Model/BodyScale.h"
#include "Model/BodyScaleSet.h"
#include "Model/BodySet.h"
#include "Model/ConstraintSet.h"
#include "Model/ContactGeometry.h"
#include "Model/ContactGeometrySet.h"
#include "Model/ContactHalfSpace.h"
#include "Model/ContactMesh.h"
#include "Model/ContactSphere.h"
#include "Model/CoordinateSet.h"
#include "Model/ElasticFoundationForce.h"
#include "Model/HuntCrossleyForce.h"
#include "Model/Ligament.h"
#include "Model/JointSet.h"
#include "Model/Marker.h"
#include "Model/MarkerSet.h"
#include "Model/PathPoint.h"
#include "Model/PathPointSet.h"
#include "Model/ConditionalPathPoint.h"
#include "Model/MovingPathPoint.h"
#include "Model/GeometryPath.h"
#include "Model/PrescribedForce.h"
#include "Model/PointToPointSpring.h"
#include "Model/ExpressionBasedPointToPointForce.h"
#include "Model/ExpressionBasedCoordinateForce.h"
#include "Model/PathSpring.h"
#include "Model/BushingForce.h"
#include "Model/FunctionBasedBushingForce.h"
#include "Model/ExpressionBasedBushingForce.h"
#include "Model/CoordinateLimitForce.h"
#include "Model/ExternalLoads.h"
#include "Model/PathActuator.h"
#include "Model/ActuatorPowerProbe.h"
#include "Model/JointInternalPowerProbe.h"
#include "Model/MuscleActiveFiberPowerProbe.h"

#include "Manager/Manager.h"

#include "Control/ControlSet.h"
#include "Control/ControlSetController.h"
#include "Control/ControlConstant.h"
#include "Control/ControlLinear.h"
#include "Control/PrescribedController.h"
#include "Wrap/PathWrap.h"
#include "Wrap/PathWrapSet.h"
#include "Wrap/WrapCylinder.h"
#include "Wrap/WrapEllipsoid.h"
#include "Wrap/WrapSphere.h"
#include "Wrap/WrapTorus.h"
#include "Wrap/WrapObjectSet.h"
#include "Wrap/WrapCylinderObst.h"
#include "Wrap/WrapSphereObst.h"
#include "Wrap/WrapDoubleCylinderObst.h"

#include "SimbodyEngine/SimbodyEngine.h"
#include "SimbodyEngine/Body.h"
#include "SimbodyEngine/Constraint.h"
#include "SimbodyEngine/WeldConstraint.h"
#include "SimbodyEngine/PointConstraint.h"
#include "SimbodyEngine/ConstantDistanceConstraint.h"
#include "SimbodyEngine/CoordinateCouplerConstraint.h"
#include "SimbodyEngine/PointOnLineConstraint.h"
#include "SimbodyEngine/EllipsoidJoint.h"
#include "SimbodyEngine/BallJoint.h"
#include "SimbodyEngine/PinJoint.h"
#include "SimbodyEngine/SliderJoint.h"
#include "SimbodyEngine/FreeJoint.h"
#include "SimbodyEngine/CustomJoint.h"
#include "SimbodyEngine/UniversalJoint.h"
#include "SimbodyEngine/GimbalJoint.h"
#include "SimbodyEngine/WeldJoint.h"
#include "SimbodyEngine/TransformAxis.h"
#include "SimbodyEngine/Coordinate.h"
#include "SimbodyEngine/SpatialTransform.h"

#include "MomentArmSolver.h"

#include "RegisterTypes_osimSimulation.h"	// to expose RegisterTypes_osimSimulation


#endif // _osimSimulation_h_
