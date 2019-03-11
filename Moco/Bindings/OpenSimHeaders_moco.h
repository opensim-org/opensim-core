#ifndef MOCO_OPENSIM_HEADERS_MOCO_H_
#define MOCO_OPENSIM_HEADERS_MOCO_H_
/* This header is only used with SWIG to create bindings.
*/

#include <Moco/osimMocoDLL.h>
#include <Moco/MocoCost/MocoCost.h>
#include <Moco/MocoWeightSet.h>
#include <Moco/MocoCost/MocoStateTrackingCost.h>
#include <Moco/MocoCost/MocoMarkerTrackingCost.h>
#include <Moco/MocoCost/MocoMarkerEndpointCost.h>
#include <Moco/MocoCost/MocoControlCost.h>
#include <Moco/MocoCost/MocoJointReactionNormCost.h>
#include <Moco/MocoBounds.h>
#include <Moco/MocoProblem.h>
#include <Moco/MocoParameter.h>
#include <Moco/MocoIterate.h>
#include <Moco/MocoTropterSolver.h>
#include <Moco/MocoCasADiSolver/MocoCasADiSolver.h>
#include <Moco/MocoTool.h>
#include <Moco/MocoInverse.h>
#include <Moco/Components/ActivationCoordinateActuator.h>
#include <Moco/Components/DeGrooteFregly2016Muscle.h>
#include <Moco/MocoUtilities.h>

#endif // MOCO_OPENSIM_HEADERS_MOCO_H_
