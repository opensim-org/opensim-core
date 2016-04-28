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

// OpenSim Common headers. Includes Simbody.
#include "OpenSim/Common/osimCommon.h"

// Leptop headers.
#include "Vendors/lepton/include/Lepton.h"

// OpenSim Simulation headers.
#include "osimSimulationDLL.h"

#include "Model/AbstractTool.h"
#include "Model/ActivationFiberLengthMuscle_Deprecated.h"
#include "Model/ActivationFiberLengthMuscle.h"
#include "Model/ActuatorForceProbe.h"
#include "Model/Actuator.h"
#include "Model/ActuatorPowerProbe.h"
#include "Model/Analysis.h"
#include "Model/AnalysisSet.h"
#include "Model/Appearance.h"
#include "Model/Bhargava2004MuscleMetabolicsProbe.h"
#include "Model/BodyScale.h"
#include "Model/BodyScaleSet.h"
#include "Model/BodySet.h"
#include "Model/BushingForce.h"
#include "Model/CMCActuatorSubsystem.h"
#include "Model/ComponentSet.h"
#include "Model/ConditionalPathPoint.h"
#include "Model/Condition.h"
#include "Model/ConstraintSet.h"
#include "Model/ContactGeometry.h"
#include "Model/ContactGeometrySet.h"
#include "Model/ContactHalfSpace.h"
#include "Model/ContactMesh.h"
#include "Model/ContactSphere.h"
#include "Model/ControllerSet.h"
#include "Model/CoordinateLimitForce.h"
#include "Model/CoordinateSet.h"
#include "Model/ElasticFoundationForce.h"
#include "Model/ExpressionBasedBushingForce.h"
#include "Model/ExpressionBasedCoordinateForce.h"
#include "Model/ExpressionBasedPointToPointForce.h"
#include "Model/ExternalForce.h"
#include "Model/ExternalLoads.h"
#include "Model/ForceAdapter.h"
#include "Model/Force.h"
#include "Model/ForceSet.h"
#include "Model/Frame.h"
#include "Model/FrameSet.h"
#include "Model/FunctionBasedBushingForce.h"
#include "Model/FunctionThresholdCondition.h"
#include "Model/Geometry.h"
#include "Model/GeometryPath.h"
#include "Model/Ground.h"
#include "Model/HuntCrossleyForce.h"
#include "Model/JointInternalPowerProbe.h"
#include "Model/JointSet.h"
#include "Model/Ligament.h"
#include "Model/Marker.h"
#include "Model/MarkerSet.h"
#include "Model/ModelComponent.h"
#include "Model/ModelComponentSet.h"
#include "Model/ModelDisplayHints.h"
#include "Model/Model.h"
#include "Model/ModelVisualizer.h"
#include "Model/ModelVisualPreferences.h"
#include "Model/MovingPathPoint.h"
#include "Model/MuscleActiveFiberPowerProbe.h"
#include "Model/Muscle.h"
#include "Model/OffsetFrame.h"
#include "Model/PathActuator.h"
#include "Model/PathPoint.h"
#include "Model/PathPointSet.h"
#include "Model/PathSpring.h"
#include "Model/PhysicalFrame.h"
#include "Model/PhysicalOffsetFrame.h"
#include "Model/PointForceDirection.h"
#include "Model/Point.h"
#include "Model/PointToPointSpring.h"
#include "Model/PrescribedForce.h"
#include "Model/Probe.h"
#include "Model/ProbeSet.h"
#include "Model/Station.h"
#include "Model/SystemEnergyProbe.h"
#include "Model/TwoFrameLinker.h"
#include "Model/Umberger2010MuscleMetabolicsProbe.h"

#include "Control/ControlConstant.h"
#include "Control/Control.h"
#include "Control/Controller.h"
#include "Control/ControlLinear.h"
#include "Control/ControlLinearNode.h"
#include "Control/ControlSetController.h"
#include "Control/ControlSet.h"
#include "Control/PrescribedController.h"
#include "Control/ToyReflexController.h"
#include "Control/TrackingController.h"

#include "Wrap/PathWrap.h"
#include "Wrap/PathWrapPoint.h"
#include "Wrap/PathWrapSet.h"
#include "Wrap/WrapCylinder.h"
#include "Wrap/WrapCylinderObst.h"
#include "Wrap/WrapDoubleCylinderObst.h"
#include "Wrap/WrapEllipsoid.h"
#include "Wrap/WrapMath.h"
#include "Wrap/WrapObject.h"
#include "Wrap/WrapObjectSet.h"
#include "Wrap/WrapResult.h"
#include "Wrap/WrapSphere.h"
#include "Wrap/WrapSphereObst.h"
#include "Wrap/WrapTorus.h"

#include "SimbodyEngine/BallJoint.h"
#include "SimbodyEngine/Body.h"
#include "SimbodyEngine/ConstantDistanceConstraint.h"
#include "SimbodyEngine/Constraint.h"
#include "SimbodyEngine/CoordinateCouplerConstraint.h"
#include "SimbodyEngine/Coordinate.h"
#include "SimbodyEngine/CustomJoint.h"
#include "SimbodyEngine/EllipsoidJoint.h"
#include "SimbodyEngine/FreeJoint.h"
#include "SimbodyEngine/GimbalJoint.h"
#include "SimbodyEngine/Joint.h"
#include "SimbodyEngine/PinJoint.h"
#include "SimbodyEngine/PlanarJoint.h"
#include "SimbodyEngine/PointConstraint.h"
#include "SimbodyEngine/PointOnLineConstraint.h"
#include "SimbodyEngine/RollingOnSurfaceConstraint.h"
#include "SimbodyEngine/SimbodyEngine.h"
#include "SimbodyEngine/SliderJoint.h"
#include "SimbodyEngine/SpatialTransform.h"
#include "SimbodyEngine/TransformAxis.h"
#include "SimbodyEngine/UnilateralConstraint.h"
#include "SimbodyEngine/UniversalJoint.h"
#include "SimbodyEngine/WeldConstraint.h"
#include "SimbodyEngine/WeldJoint.h"

#include "Manager/Manager.h"

#include "RegisterTypes_osimSimulation.h" 

#include "AssemblySolver.h"
#include "CoordinateReference.h"
#include "InverseDynamicsSolver.h"
#include "InverseKinematicsSolver.h"
#include "MarkersReference.h"
#include "MomentArmSolver.h"
#include "osimSimulationDLL.h"
#include "osimSimulation.h"
#include "Reference.h"
#include "RegisterTypes_osimSimulation.h"
#include "Solver.h"
#include "StatesTrajectory.h"
#include "StatesTrajectoryReporter.h"

// Standard headers. It is okay to be redundant here. Include guards to the
// rescue.
#include <string>
#include <cstring>
#include <cctype>
#include <iostream>
#include <exception>
#include <cstdio>
#include <memory>
#include <sstream>

#endif // _osimSimulation_h_
