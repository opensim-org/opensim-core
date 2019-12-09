/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleMocoInverse.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/// This example features two different tracking problems solved using the
/// MocoTrack tool. 
///  - The first problem demonstrates the basic usage of the tool interface
///    to solve a torque-driven marker tracking problem. 
///  - The second problem shows how to customize a muscle-driven state tracking 
///    problem using more advanced features of the tool interface.
/// 
/// Data and model source: https://simtk.org/projects/full_body
///
/// TODO move to the readme.
/// Model
/// -----
/// The model described in the file 'subject_walk_armless.osim' included in this
/// file is a modified version of the Rajagopoal et al. 2016 musculoskeletal 
/// model. The lumbar, subtalar, and mtp coordinates have been replaced with
/// WeldJoint%s and residual actuators have been added to the pelvis (1 N-m for
/// rotational coordinates and 10 N for translational coordinates). Finally, the
/// arms and all associated components have been removed for simplicity.
/// 
/// Data
/// ----
/// The coordinate and marker data included in the 'coordinates.sto' and 
/// 'marker_trajectories.trc' files also come from the Rajagopal et al. 2016
/// model distribution. The coordinates were computed using inverse kinematics
/// and modified via the Residual Reduction Algorithm (RRA). 

#include <Moco/osimMoco.h>

using namespace OpenSim;

int main() {

    // TODO.

    return EXIT_SUCCESS;
}
