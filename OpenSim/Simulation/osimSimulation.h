#ifndef _osimSimulation_h_
#define _osimSimulation_h_
// osimSimulation.h
// author: Ayman Habib
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005-12, Stanford University. All rights reserved. 
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
#include "Model/BushingForce.h"
#include "Model/ExternalLoads.h"

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
#include "SimbodyEngine/WeldJoint.h"
#include "SimbodyEngine/TransformAxis.h"
#include "SimbodyEngine/Coordinate.h"
#include "SimbodyEngine/SpatialTransform.h"
#include "RegisterTypes_osimSimulation.h"	// to expose RegisterTypes_osimSimulation

#endif _osimSimulation_h_
