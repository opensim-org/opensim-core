#ifndef OPENSIM_OPENSIM_HEADERS_MOCO_H_
#define OPENSIM_OPENSIM_HEADERS_MOCO_H_
/* This header is only used with SWIG to create bindings.
 */

#include <OpenSim/Moco/About.h>
#include <OpenSim/Moco/MocoBounds.h>
#include <OpenSim/Moco/MocoCasADiSolver/MocoCasADiSolver.h>
#include <OpenSim/Moco/MocoControlBoundConstraint.h>
#include <OpenSim/Moco/MocoFrameDistanceConstraint.h>
#include <OpenSim/Moco/MocoOutputConstraint.h>
#include <OpenSim/Moco/MocoGoal/MocoAccelerationTrackingGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoContactTrackingGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoContactImpulseTrackingGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoControlGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoControlTrackingGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoInitialActivationGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoInitialForceEquilibriumDGFGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoInitialVelocityEquilibriumDGFGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoJointReactionGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoMarkerFinalGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoMarkerTrackingGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoOrientationTrackingGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoAngularVelocityTrackingGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoOutputGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoPeriodicityGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoStateTrackingGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoSumSquaredStateGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoTranslationTrackingGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoStepTimeAsymmetryGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoStepLengthAsymmetryGoal.h>
#include <OpenSim/Moco/MocoInverse.h>
#include <OpenSim/Moco/MocoParameter.h>
#include <OpenSim/Moco/MocoProblem.h>
#include <OpenSim/Moco/MocoStudy.h>
#include <OpenSim/Moco/MocoStudyFactory.h>
#include <OpenSim/Moco/MocoTrack.h>
#include <OpenSim/Moco/MocoTrajectory.h>
#include <OpenSim/Moco/MocoTropterSolver.h>
#include <OpenSim/Moco/MocoUtilities.h>
#include <OpenSim/Moco/MocoWeightSet.h>
#include <OpenSim/Moco/ModelOperatorsDGF.h>
#include <OpenSim/Moco/MocoScaleFactor.h>
#include <OpenSim/Moco/osimMocoDLL.h>

#endif // OPENSIM_OPENSIM_HEADERS_MOCO_H_
