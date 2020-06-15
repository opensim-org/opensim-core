/* -------------------------------------------------------------------------- *
 * OpenSim Moco: INDYGO.cpp                                                   *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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

#include "INDYGOProblemInternal.h"

// TODO to get a good initial guess: solve without pennation! avoid all those
//         sqrts!


using namespace OpenSim;

void INDYGO::Solution::write(const std::string& prefix) const
{
    auto write = [&](const TimeSeriesTable& table, const std::string& suffix)
    {
        if (table.getNumRows()) {
            STOFileAdapter_<double>::write(table,
                                           prefix + "_" + suffix + ".sto");
        }
    };
    write(excitation, "excitation");
    write(activation, "activation");
    write(norm_fiber_length, "norm_fiber_length");
    write(norm_fiber_velocity, "norm_fiber_velocity");
    write(other_controls, "other_controls");
    write(tendon_force, "tendon_force");
    write(norm_tendon_force, "norm_tendon_force");
    write(tendon_force_rate_control, "tendon_force_rate_control");
}

INDYGO::INDYGO() {
    constructProperties();
}

INDYGO::INDYGO(
        const std::string& setupFilePath) : InverseMuscleSolver(setupFilePath) {
    constructProperties();
    updateFromXMLDocument();
}

void INDYGO::constructProperties() {
    constructProperty_initial_guess("static_optimization");
    constructProperty_zero_initial_activation(false);
    constructProperty_fiber_dynamics_mode("fiber_length");
    constructProperty_activation_dynamics_mode("explicit");
}

INDYGO::Solution INDYGO::solve() const {

    // TODO make sure experimental data is available for all unconstrained
    // coordinates; throw exception otherwise!

    // Load model and kinematics files.
    // --------------------------------
    Model origModel; // without reserve actuators.
    TimeSeriesTable kinematics;
    TimeSeriesTable netGeneralizedForces;
    loadModelAndData(origModel, kinematics, netGeneralizedForces);

    // Decide the coordinates for which net generalized forces will be achieved.
    // -------------------------------------------------------------------------
    // TODO move to InverseMuscleSolver
    Model model(origModel);
    SimTK::State state = model.initSystem();
    std::vector<const Coordinate*> coordsToActuate;
    const auto coordsInOrder = model.getCoordinatesInMultibodyTreeOrder();
    const auto modelPath = model.getAbsolutePath();
    if (!getProperty_coordinates_to_include().empty()) {
        // Our goal is to create a list of Coordinate* in multibody tree order.
        // We will keep track of which requested coordinates we actually find.
        std::set<std::string> coordsToInclude;
        auto numCoordsToInclude = getProperty_coordinates_to_include().size();
        for (int iInclude = 0; iInclude < numCoordsToInclude; ++iInclude) {
            coordsToInclude.insert(get_coordinates_to_include(iInclude));
        }
        // Go through coordinates in order.
        for (auto& coord : coordsInOrder) {
            // Remove the model name from the coordinate path.
            const auto coordPath = coord->getAbsolutePathString();
            // Should this coordinate be included?
            const auto foundCoordPath = coordsToInclude.find(coordPath);
            if (foundCoordPath != coordsToInclude.end()) {
                OPENSIM_THROW_IF_FRMOBJ(coord->isConstrained(state), Exception,
                        format("Coordinate '%s' is constrained and "
                                "thus cannot be listed under "
                                "'coordinates_to_include'.", coordPath));
                coordsToActuate.push_back(coord.get());
                // No longer need to search for this coordinate.
                coordsToInclude.erase(foundCoordPath);
            }
        }
        // Any remaining "coordsToInclude" are not in the model.
        if (!coordsToInclude.empty()) {
            std::string msg = "Could not find the following coordinates "
                    "listed under 'coordinates_to_include' (make sure to "
                    "use the *path* to the coordinate):\n";
            for (const auto& coordPath : coordsToInclude) {
                msg += "  " + coordPath + "\n";
            }
            OPENSIM_THROW_FRMOBJ(Exception, msg);
        }
    } else {
        // User did not specify coords to include, so include all of them.
        for (auto& coord : model.getCoordinatesInMultibodyTreeOrder()) {
            if (!coord->isConstrained(state)) {
                coordsToActuate.push_back(coord.get());
            }
        }
    }
    std::cout << "The following Coordinates will be actuated:" << std::endl;
    for (const auto* coord : coordsToActuate) {
        std::cout << "  " << coord->getAbsolutePathString() << std::endl;
    }

    // Process which actuators are included.
    // -------------------------------------
    processActuatorsToInclude(model);

    // Ensure that all enabled actuators are supported.
    // ------------------------------------------------
    auto actuators = model.getComponentList<ScalarActuator>();
    for (const auto& actuator : actuators) {
        if (actuator.get_appliesForce() &&
                !dynamic_cast<const Muscle*>(&actuator) &&
                !dynamic_cast<const CoordinateActuator*>(&actuator)) {
            throw std::runtime_error("[INDYGO] Only Muscles and "
                    "CoordinateActuators are currently supported but the"
                    " model contains an enabled "
                    + actuator.getConcreteClassName() + ". Either set "
                    "appliesForce=false for this actuator, or remove it "
                    "from the model.");
        }
    }

    // Create reserve actuators.
    // -------------------------
    if (get_create_reserve_actuators() != -1) {
        const auto& optimalForce = get_create_reserve_actuators();
        OPENSIM_THROW_IF(optimalForce <= 0, Exception,
                format("Invalid value (%g) for create_reserve_actuators; "
                       "should be -1 or positive.", optimalForce));

        std::cout << "Adding reserve actuators with an optimal force of "
                  << optimalForce << "..." << std::endl;

        std::vector<std::string> coordPaths;
        // Borrowed from CoordinateActuator::CreateForceSetOfCoordinateAct...
        for (const auto* coord : coordsToActuate) {
            auto* actu = new CoordinateActuator();
            actu->setCoordinate(const_cast<Coordinate*>(coord));
            auto path = coord->getAbsolutePathString();
            coordPaths.push_back(path);
            // Get rid of model name.
            path = coord->getAbsolutePathString();
            // Get rid of slashes in the path; slashes not allowed in names.
            std::replace(path.begin(), path.end(), '/', '_');
            actu->setName("reserve_" + path);
            actu->setOptimalForce(optimalForce);
            model.addComponent(actu);
        }
        // Re-make the system, since there are new actuators.
        model.initSystem();
        std::cout << "Added " << coordPaths.size() << " reserve actuator(s), "
                "for each of the following coordinates:" << std::endl;
        for (const auto& name : coordPaths) {
            std::cout << "  " << name << std::endl;
        }
    }

    // Determine initial and final times.
    // ----------------------------------
    // TODO create tests for initial_time and final_time.
    // TODO check for errors in initial_time and final_time.
    double initialTime;
    double finalTime;
    int numMeshPoints;
    OPENSIM_THROW_IF(get_mesh_point_frequency() <= 0, Exception,
            format("Invalid value (%g) for mesh_point_frequency; "
                   "must be positive.",
                    get_mesh_point_frequency()));
    determineInitialAndFinalTimes(kinematics, netGeneralizedForces,
            get_mesh_point_frequency(),
            initialTime, finalTime, numMeshPoints);

    // Process experimental data.
    // --------------------------
    // TODO move to InverseMuscleSolver
    InverseMuscleSolverMotionData motionData;
    OPENSIM_THROW_IF(
            get_lowpass_cutoff_frequency_for_kinematics() <= 0 &&
            get_lowpass_cutoff_frequency_for_kinematics() != -1,
            Exception,
            "Invalid value for cutoff frequency for kinematics.");
    OPENSIM_THROW_IF(
            get_lowpass_cutoff_frequency_for_joint_moments() <= 0 &&
            get_lowpass_cutoff_frequency_for_joint_moments() != -1,
            Exception,
            "Invalid value for cutoff frequency for joint moments.");
    if (netGeneralizedForces.getNumRows()) {
        motionData = InverseMuscleSolverMotionData(model, coordsToActuate,
                initialTime, finalTime, kinematics,
                get_lowpass_cutoff_frequency_for_kinematics(),
                get_lowpass_cutoff_frequency_for_joint_moments(),
                netGeneralizedForces);
    } else {
        // We must perform inverse dynamics.
        motionData = InverseMuscleSolverMotionData(model, coordsToActuate,
                initialTime, finalTime, kinematics,
                get_lowpass_cutoff_frequency_for_kinematics(),
                get_lowpass_cutoff_frequency_for_joint_moments());
    }

    // Check for errors in formulation modes.
    // --------------------------------------
    OPENSIM_THROW_IF(get_fiber_dynamics_mode() != "fiber_length" &&
                     get_fiber_dynamics_mode() != "tendon_force",
            Exception,
            format("Invalid value (%s) for "
                   "fiber_dynamics_mode; should be 'fiber_length' or "
                   "'tendon_force'.", get_fiber_dynamics_mode()));
    OPENSIM_THROW_IF(get_activation_dynamics_mode() != "explicit" &&
                     get_activation_dynamics_mode() != "implicit",
            Exception,
            format("Invalid value (%s) "
                   "for activation_dynamics_mode; should be 'explicit' or "
                   "'implicit'.", get_activation_dynamics_mode()));
    OPENSIM_THROW_IF(get_activation_dynamics_mode() == "implicit", Exception,
            "Implicit activation dynamics is not supported yet.");

    // Create the optimal control problem.
    // -----------------------------------
    auto ocp = std::make_shared<INDYGOProblemInternal<adouble>>(*this,
            model, motionData, get_fiber_dynamics_mode());

    // Solve for an initial guess with static optimization.
    // ----------------------------------------------------
    OPENSIM_THROW_IF(get_initial_guess() != "bounds" &&
                     get_initial_guess() != "static_optimization", Exception,
            format("Invalid value (%s) for "
                   "initial_guess; should be 'static_optimization' or 'bounds'.",
                    get_initial_guess()));
    tropter::Iterate guess;
    if (get_initial_guess() == "static_optimization") {
        std::cout << std::string(79, '=') << std::endl;
        std::cout << "Computing an initial guess with static optimization."
                  << std::endl;
        std::cout << std::string(79, '-') << std::endl;
        GlobalStaticOptimization gso;
        gso.setModel(origModel);
        gso.setKinematicsData(kinematics);
        gso.set_lowpass_cutoff_frequency_for_kinematics(
                get_lowpass_cutoff_frequency_for_kinematics());
        if (netGeneralizedForces.getNumRows()) {
            gso.setNetGeneralizedForcesData(netGeneralizedForces);
        }
        gso.set_lowpass_cutoff_frequency_for_joint_moments(
                get_lowpass_cutoff_frequency_for_joint_moments());
        gso.set_create_reserve_actuators(get_create_reserve_actuators());
        if (!getProperty_initial_time().empty()) {
            gso.set_initial_time(get_initial_time());
        }
        if (!getProperty_final_time().empty()) {
            gso.set_final_time(get_final_time());
        }
        gso.set_mesh_point_frequency(get_mesh_point_frequency());
        if (!getProperty_coordinates_to_include().empty()) {
            gso.set_coordinates_to_include(
                    getProperty_coordinates_to_include());
        }
        if (!getProperty_actuators_to_include().empty()) {
            gso.set_actuators_to_include(
                    getProperty_actuators_to_include());
        }
        // Convert the static optimization solution into a guess.
        GlobalStaticOptimization::Solution gsoSolution = gso.solve();
        // gsoSolution.write("DEBUG_INDYGO_GSO_solution");
        INDYGO::Solution mrsGuess;
        mrsGuess.excitation = gsoSolution.activation;
        mrsGuess.activation = gsoSolution.activation;
        mrsGuess.norm_fiber_length = gsoSolution.norm_fiber_length;
        mrsGuess.norm_fiber_velocity = gsoSolution.norm_fiber_velocity;
        mrsGuess.other_controls = gsoSolution.other_controls;
        mrsGuess.tendon_force = gsoSolution.tendon_force;
        mrsGuess.norm_tendon_force = gsoSolution.norm_tendon_force;
        // We do not have a guess for this control variable, so we set it to
        // zero (after getting a table with the correct dimensions).
        mrsGuess.tendon_force_rate_control = gsoSolution.norm_tendon_force;
        mrsGuess.tendon_force_rate_control.updMatrix().setToZero();
        guess = ocp->construct_iterate(mrsGuess);
        // guess.write("DEBUG_INDYGO_guess.csv");
    } else {
        // This is the behavior if no guess is provided.
        std::cout << std::string(79, '=') << std::endl;
        std::cout << "Using variable bounds to form initial guess."
                  << std::endl;
        std::cout << std::string(79, '-') << std::endl;
    }

    // TODO perhaps solve without pennation initially? Pennation slows down
    // convergence.

    // Solve the optimal control problem.
    // ----------------------------------
    ocp->print_description();
    tropter::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
            "ipopt", numMeshPoints - 1);
    // TODO Consider trying using the quasi-Newton mode; it seems to work
    // well for some problems but not well for larger problems.
    // dircol.get_opt_solver().set_hessian_approximation("limited-memory");
    std::cout << std::string(79, '=') << std::endl;
    std::cout << "Running the Muscle Redundancy Solver." << std::endl;
    std::cout << std::string(79, '-') << std::endl;
    tropter::Solution ocp_solution;
    if (get_initial_guess() == "static_optimization") {
        ocp_solution = dircol.solve(guess);
    } else if (get_initial_guess() == "bounds") {
        ocp_solution = dircol.solve();
    } else {
        OPENSIM_THROW_FRMOBJ(Exception,
                format("Expected 'initial_guess' property "
                       "to be 'static_optimization or 'bounds', but got '%s'.",
                        get_initial_guess()));
    }

    // Return the solution.
    // --------------------
    // TODO remove:
    ocp_solution.write("INDYGO_OCP_solution.csv");
    // dircol.print_constraint_values(ocp_solution);
    Solution solution = ocp->deconstruct_iterate(ocp_solution);
    if (get_write_solution() != "false") {
        IO::makeDir(get_write_solution());
        std::string prefix = getName().empty() ? "INDYGO" : getName();
        solution.write(get_write_solution() +
                SimTK::Pathname::getPathSeparator() + prefix +
                "_solution");
    }
    return solution;
}
