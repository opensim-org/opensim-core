#ifndef OPENSIM_AUXILIARY_TEST_MUSCLE_FUNCTIONS_H_
#define OPENSIM_AUXILIARY_TEST_MUSCLE_FUNCTIONS_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  auxiliaryTestMuscleFunctions.h                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include "OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h"

/** A debugging utility for investigating muscle equilibrium failures.
    For a given muscle at a given state report how the muscle fiber and
    tendon force varies with fiber-length. Also, report the difference, 
    which represents the function that the muscle equilibrium solver is
    trying to find a root (zero) for. The intended use is to invoke this
    method when the muscle fails to compute the equilibrium fiber-length,
    so that one can plot the equilibrium force error vs. fiber-length
    to help diagnose the cause of the failure. The force-velocity 
    multiplier is assumed to be 1.0 (e.g. static fiber) unless otherwise
    specified.*/
template <typename T = OpenSim::ActivationFiberLengthMuscle>
void reportTendonAndFiberForcesAcrossFiberLengths(const T& muscle,
    const SimTK::State& state, const double fiberVelocityMultiplier = 1.0)
{
    // should only be using this utility for equilibrium muscles 
    // with a compliant tendon
    assert(!muscle.get_ignore_tendon_compliance());

    SimTK::State s = state;

    OpenSim::DataTable_<double, double> forcesVsFiberLengthTable;
    std::vector<std::string> labels{ "fiber_length", "pathLength",
        "tendon_force", "fiber_force", "activation", "activeFiberForce",
        "passiveFiberForce", "equilibriumError" };
    forcesVsFiberLengthTable.setColumnLabels(labels);

    // Constants
    const int N = 100;
    const int nc = int(labels.size());

    const double maxFiberLength = 2.0*muscle.getOptimalFiberLength();
    const double minFiberLength = muscle.getMinimumFiberLength();
    const double dl = (maxFiberLength - minFiberLength) / N;
    const double fiso = muscle.getMaxIsometricForce();

    // Variables
    double fiberLength = SimTK::NaN;
    // double vmt = SimTK::NaN;
    double tendonForce = SimTK::NaN;
    double activeFiberForce = SimTK::NaN;
    double passiveFiberForce = SimTK::NaN;
    double cosphi = SimTK::NaN;
    double flm = SimTK::NaN;
    double a = SimTK::NaN;

    SimTK::RowVector row(nc, SimTK::NaN);
    for (int i = 0; i <= N; ++i) {
        fiberLength = minFiberLength + i*dl;
        s.setTime(fiberLength);
        muscle.setFiberLength(s, fiberLength);
        muscle.getModel().realizeDynamics(s);

        // vmt = muscle.getSpeed(s);

        tendonForce = muscle.getTendonForce(s);

        a = muscle.getActivation(s);

        flm = muscle.getActiveForceLengthMultiplier(s);
        cosphi = muscle.getCosPennationAngle(s);

        activeFiberForce = a*fiso*flm*fiberVelocityMultiplier;

        passiveFiberForce = muscle.getPassiveFiberForce(s);

        row[0] = fiberLength; // muscle.getFiberLength(s);
        row[1] = muscle.getLength(s);
        row[2] = tendonForce;
        row[3] = (activeFiberForce + passiveFiberForce)*cosphi;
        row[4] = muscle.getActivation(s);
        row[5] = activeFiberForce;
        row[6] = passiveFiberForce;
        row[7] = row[3] - row[2];

        forcesVsFiberLengthTable.appendRow(s.getTime(), row);
    }

    std::string fileName = "forcesVsFiberLength_"
        + std::to_string(a) + ".sto";

    OpenSim::STOFileAdapter::write(forcesVsFiberLengthTable, fileName);
}

#endif // OPENSIM_AUXILIARY_TEST_MUSCLE_FUNCTIONS_H_
