#ifndef MOCO_OPENSIM_HEADERS_MOCO_H_
#define MOCO_OPENSIM_HEADERS_MOCO_H_
/* This header is only used with SWIG to create bindings.
 */

#include <Moco/About.h>
#include <Moco/Common/TableProcessor.h>
#include <Moco/Components/DeGrooteFregly2016Muscle.h>
#include <Moco/Components/ModelFactory.h>
#include <Moco/Components/MultivariatePolynomialFunction.h>
#include <Moco/Components/PositionMotion.h>
#include <Moco/MocoBounds.h>
#include <Moco/MocoCasADiSolver/MocoCasADiSolver.h>
#include <Moco/MocoControlBoundConstraint.h>
#include <Moco/MocoFrameDistanceConstraint.h>
#include <Moco/MocoGoal/MocoContactTrackingGoal.h>
#include <Moco/MocoGoal/MocoControlGoal.h>
#include <Moco/MocoGoal/MocoControlTrackingGoal.h>
#include <Moco/MocoGoal/MocoGoal.h>
#include <Moco/MocoGoal/MocoInitialActivationGoal.h>
#include <Moco/MocoGoal/MocoInitialForceEquilibriumGoal.h>
#include <Moco/MocoGoal/MocoInitialVelocityEquilibriumDGFGoal.h>
#include <Moco/MocoGoal/MocoJointReactionGoal.h>
#include <Moco/MocoGoal/MocoMarkerFinalGoal.h>
#include <Moco/MocoGoal/MocoMarkerTrackingGoal.h>
#include <Moco/MocoGoal/MocoOrientationTrackingGoal.h>
#include <Moco/MocoGoal/MocoAccelerationTrackingGoal.h>
#include <Moco/MocoGoal/MocoOutputGoal.h>
#include <Moco/MocoGoal/MocoPeriodicityGoal.h>
#include <Moco/MocoGoal/MocoStateTrackingGoal.h>
#include <Moco/MocoGoal/MocoSumSquaredStateGoal.h>
#include <Moco/MocoGoal/MocoTranslationTrackingGoal.h>
#include <Moco/MocoInverse.h>
#include <Moco/MocoParameter.h>
#include <Moco/MocoProblem.h>
#include <Moco/MocoStudy.h>
#include <Moco/MocoStudyFactory.h>
#include <Moco/MocoTrack.h>
#include <Moco/MocoTrajectory.h>
#include <Moco/MocoTropterSolver.h>
#include <Moco/MocoUtilities.h>
#include <Moco/MocoWeightSet.h>
#include <Moco/ModelOperators.h>
#include <Moco/osimMocoDLL.h>

#endif // MOCO_OPENSIM_HEADERS_MOCO_H_
