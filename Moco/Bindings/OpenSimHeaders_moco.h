#ifndef MOCO_OPENSIM_HEADERS_MOCO_H_
#define MOCO_OPENSIM_HEADERS_MOCO_H_
/* This header is only used with SWIG to create bindings.
 */

#include <Moco/Common/TableProcessor.h>
#include <Moco/Components/ActivationCoordinateActuator.h>
#include <Moco/Components/DeGrooteFregly2016Muscle.h>
#include <Moco/Components/ModelFactory.h>
#include <Moco/Components/PositionMotion.h>
#include <Moco/Components/SmoothSphereHalfSpaceForce.h>
#include <Moco/Components/MultivariatePolynomialFunction.h>
#include <Moco/MocoBounds.h>
#include <Moco/MocoCasADiSolver/MocoCasADiSolver.h>
#include <Moco/MocoControlBoundConstraint.h>
#include <Moco/MocoCost/MocoControlCost.h>
#include <Moco/MocoCost/MocoCost.h>
#include <Moco/MocoCost/MocoJointReactionCost.h>
#include <Moco/MocoCost/MocoMarkerFinalCost.h>
#include <Moco/MocoCost/MocoMarkerTrackingCost.h>
#include <Moco/MocoCost/MocoOrientationTrackingCost.h>
#include <Moco/MocoCost/MocoStateTrackingCost.h>
#include <Moco/MocoCost/MocoSumSquaredStateCost.h>
#include <Moco/MocoCost/MocoTranslationTrackingCost.h>
#include <Moco/MocoInverse.h>
#include <Moco/MocoParameter.h>
#include <Moco/MocoProblem.h>
#include <Moco/MocoStudy.h>
#include <Moco/MocoTrack.h>
#include <Moco/MocoTrajectory.h>
#include <Moco/MocoTropterSolver.h>
#include <Moco/MocoUtilities.h>
#include <Moco/MocoWeightSet.h>
#include <Moco/ModelOperators.h>
#include <Moco/osimMocoDLL.h>

#endif // MOCO_OPENSIM_HEADERS_MOCO_H_
