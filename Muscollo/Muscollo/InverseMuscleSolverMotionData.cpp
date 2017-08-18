
#include "InverseMuscleSolverMotionData.h"

#include <tropter.h>

#include <OpenSim/OpenSim.h>
// TODO should not be needed after updating to a newer OpenSim:
#include <OpenSim/Simulation/InverseDynamicsSolver.h>

using namespace OpenSim;

/// Given a table, create a spline for each column in `labels`, and provide all
/// of these splines in a set.
/// If `labels` is empty, all columns are splined.
/// This function exists because GCVSplineSet's constructor takes a Storage,
/// not a TimeSeriesTable.
GCVSplineSet createGCVSplineSet(const TimeSeriesTable& table,
                                const std::vector<std::string>& labels = {},
                                int degree = 5,
                                double errorVariance = 0.0) {
    GCVSplineSet set;
    const auto& time = table.getIndependentColumn();
    auto labelsToUse = labels;
    if (labelsToUse.empty()) labelsToUse = table.getColumnLabels();
    for (const auto& label : labelsToUse) {
        const auto& column = table.getDependentColumn(label);
        set.adoptAndAppend(new GCVSpline(degree, column.size(), time.data(),
                                         &column[0], label, errorVariance));
    }
    return set;
}

InverseMuscleSolverMotionData::InverseMuscleSolverMotionData(
        const Model& model,
        const std::vector<const Coordinate*>& coordsToActuate,
        const double& initialTime, const double& finalTime,
        const TimeSeriesTable& kinematicsData,
        const double& lowpassCutoffKinematics) :
        _coordPathsToActuate(createCoordPathsToActuate(model, coordsToActuate)),
        _initialTime(initialTime), _finalTime(finalTime) {
    // TODO spline/filter over only [initial_time, final_time]?

    _numCoordsToActuate = coordsToActuate.size();

    // Muscle analysis.
    // ================
    // Form a StatesTrajectory.
    // ------------------------
    // (TODO The parameter to this constructor should be a StatesTrajectory).
    Storage statesSto;
    OpenSim::Array<std::string> statesLabels("time", 1); // first column label.
    for (const auto& label : kinematicsData.getColumnLabels()) {
        statesLabels.append(label);
    }
    statesSto.setColumnLabels(statesLabels);
    double timeSlop = 0.05;
    for (size_t i_time = 0; i_time < kinematicsData.getNumRows(); ++i_time) {
        const auto& time = kinematicsData.getIndependentColumn()[i_time];
        SimTK::Vector row = kinematicsData.getRowAtIndex(i_time).transpose();
        // Only append columns approximately within [initialTime, finalTime].
        if (initialTime - timeSlop <= time && time <= finalTime + timeSlop) {
            statesSto.append(time, row);
        }
    }
    const double& cutoffFrequency = lowpassCutoffKinematics;
    if (cutoffFrequency > 0) {
        // Filtering kinematics is important for obtaining muscle activations
        // that are not noisy. Also, smooth kinematics improves the
        // convergence time of the optimization.
        // We pad because OpenSim's other tools also pad; not clear if it's
        // really necessary.
        statesSto.pad(statesSto.getSize()/2);
        statesSto.lowpassIIR(cutoffFrequency);
    }
    auto statesTraj = StatesTrajectory::createFromStatesStorage(model,
            statesSto,
            true, false);
    // TODO give an error if the states do not contain generalized speeds.

    // Compute muscle quantities and spline the data.
    // ----------------------------------------------
    TimeSeriesTable muscleTendonLengths;
    // TimeSeriesTable muscleTendonVelocities; using MTL spline now.
    // TODO get list of muscles from INDYGO.
    const auto muscleList = model.getComponentList<Muscle>();
    std::vector<const Muscle*> activeMuscles;
    std::vector<std::string> musclePathNames;
    for (const auto& muscle : muscleList) {
        if (muscle.get_appliesForce()) {
            musclePathNames.push_back(muscle.getAbsolutePathName());
            activeMuscles.push_back(&muscle);
        }
    }
    _numActiveMuscles = activeMuscles.size();
    if (!musclePathNames.empty()) {

        // Muscle-tendon lengths and velocities.
        // `````````````````````````````````````
        muscleTendonLengths.setColumnLabels(musclePathNames);
        // muscleTendonVelocities.setColumnLabels(musclePathNames);
        SimTK::RowVector rowMTL(musclePathNames.size());
        // SimTK::RowVector rowMTV(musclePathNames.size());
        for (size_t i_time = 0; i_time < statesTraj.getSize(); ++i_time) {
            const auto& state = statesTraj[i_time];
            model.realizeVelocity(state);

            // TODO handle disabled muscles. (This might be outdated now).
            int i_muscle = 0;
            for (const auto* muscle : activeMuscles) {
                rowMTL[i_muscle] = muscle->getLength(state);
                // rowMTV[i_muscle] = muscle->getLengtheningSpeed(state);
                i_muscle++;
            }
            muscleTendonLengths.appendRow(state.getTime(), rowMTL);
            // muscleTendonVelocities.appendRow(state.getTime(), rowMTV);
        }
        _muscleTendonLengths = createGCVSplineSet(muscleTendonLengths);
        // auto MTLSto = std::unique_ptr<Storage>(
        //         _muscleTendonLengths.constructStorage(0));
        // MTLSto->print("DEBUG_muscle_tendon_lengths.sto");


        // TODO Separately splining muscleTendonLengths and velocities might
        // lead to inconsistency.
        // _muscleTendonVelocities = createGCVSplineSet(muscleTendonVelocities);

        // Moment arms.
        // ````````````
        // TODO see above; _nu
        // This member holds moment arms across time, DOFs, and muscles.
        _momentArms.resize(_numCoordsToActuate);
        // Working memory.
        SimTK::RowVector rowMA(musclePathNames.size());
        for (size_t i_dof = 0; i_dof < _numCoordsToActuate; ++i_dof) {
            TimeSeriesTable momentArmsThisDOF;
            momentArmsThisDOF.setColumnLabels(musclePathNames);
            for (size_t i_time = 0; i_time < statesTraj.getSize(); ++i_time) {
                const auto& state = statesTraj[i_time];
                int i_muscle = 0;
                for (const auto* muscle : activeMuscles) {
                    rowMA[i_muscle] = muscle->computeMomentArm(state,
                            const_cast<Coordinate&>(*coordsToActuate[i_dof]));
                    i_muscle++;
                }
                momentArmsThisDOF.appendRow(state.getTime(), rowMA);
            }
            CSVFileAdapter::write(momentArmsThisDOF,
                    "DEBUG_momentArmsThisDOF.csv");
            _momentArms[i_dof] = createGCVSplineSet(momentArmsThisDOF);
        }
    }
}

InverseMuscleSolverMotionData::InverseMuscleSolverMotionData(
        const Model& model,
        const std::vector<const Coordinate*>& coordsToActuate,
        const double& initialTime, const double& finalTime,
        const TimeSeriesTable& kinematicsData,
        const double& lowpassCutoffKinematics,
        const double& lowpassCutoffJointMoments) :
        InverseMuscleSolverMotionData(model, coordsToActuate,
                initialTime, finalTime, kinematicsData,
                lowpassCutoffKinematics) {
    // Inverse dynamics.

    // All that computeInverseDynamics needs is the paths of the coordinates.
    // Furthermore, computeInverseDynamics uses a copy of the model, so it does
    // not make sense to pass in the Coordinate* from the original model.
    computeInverseDynamics(model, _coordPathsToActuate, kinematicsData,
            lowpassCutoffJointMoments);
}

InverseMuscleSolverMotionData::InverseMuscleSolverMotionData(
        const Model& model,
        const std::vector<const Coordinate*>& coordsToActuate,
        const double& initialTime, const double& finalTime,
        const TimeSeriesTable& kinematicsData,
        const double& lowpassCutoffKinematics,
        const TimeSeriesTable& netGeneralizedForcesData) :
        InverseMuscleSolverMotionData(model, coordsToActuate,
                initialTime, finalTime, kinematicsData, lowpassCutoffKinematics)
{
    // Inverse dynamics.
    // Only store the columns corresponding to coordsToActuate, and store
    // them in multibody tree order.
    // TODO better error handling for invalid column labels.
    try {
        _netGeneralizedForces = createGCVSplineSet(netGeneralizedForcesData,
                _coordPathsToActuate);
    } catch (KeyNotFound& ex) {
        // The net generalized forces file's column labels do not seem to
        // be coordinate paths. Try using the column label format produced
        // by inverse dynamics (e.g., "knee_flexion_r_moment").
        std::cout << "Column labels of net generalized forces file do not "
                "match paths of coordinates to include. Trying to parse "
                "column labels using Inverse Dynamics Tool format...";
        std::vector<std::string> invDynFormat(coordsToActuate.size());
        size_t iCoord = 0;
        for (const auto* coord : coordsToActuate) {
            invDynFormat[iCoord] = coord->getName() + (
                    coord->getMotionType() == Coordinate::Rotational ?
                    "_moment" : "_force");
            ++iCoord;
        }
        _netGeneralizedForces = createGCVSplineSet(netGeneralizedForcesData,
                invDynFormat);
        std::cout << "success!" << std::endl;
    }
}

void InverseMuscleSolverMotionData::interpolateNetGeneralizedForces(
        const Eigen::VectorXd& times,
        Eigen::MatrixXd& desiredMoments) const {
    OPENSIM_THROW_IF(times[0] < getInitialTime(), Exception,
            "Requested initial time is lower than permitted.");
    OPENSIM_THROW_IF(times[times.size()-1] < getFinalTime(), Exception,
            "Requested final time is greater than permitted.");

    desiredMoments.resize(_netGeneralizedForces.getSize(), times.size());
    for (size_t i_time = 0; i_time < size_t(times.size()); ++i_time) {
        for (size_t i_dof = 0; i_dof < size_t(_netGeneralizedForces.getSize());
             ++i_dof) {
            const double value = _netGeneralizedForces[i_dof].calcValue(
                    SimTK::Vector(1, times[i_time]));
            desiredMoments(i_dof, i_time) = value;
        }
    }
    tropter::write(times, desiredMoments, "DEBUG_desiredMoments.csv");
}

void InverseMuscleSolverMotionData::interpolateMuscleTendonLengths(
        const Eigen::VectorXd& times,
        Eigen::MatrixXd& muscleTendonLengths) const {
    OPENSIM_THROW_IF(times[0] < getInitialTime(), Exception,
            "Requested initial time is lower than permitted.");
    OPENSIM_THROW_IF(times[times.size()-1] < getFinalTime(), Exception,
            "Requested final time is greater than permitted.");

    muscleTendonLengths.resize(_numActiveMuscles, times.size());
    // The matrix is in column-major format.
    for (size_t i_mesh = 0; i_mesh < size_t(times.size()); ++i_mesh) {
        SimTK::Vector time(1, times[i_mesh]);
        for (size_t i_mus = 0; i_mus < _numActiveMuscles; ++i_mus) {
            muscleTendonLengths(i_mus, i_mesh) =
                    _muscleTendonLengths.get(i_mus).calcValue(time);
        }
    }
}

void InverseMuscleSolverMotionData::interpolateMuscleTendonVelocities(
        const Eigen::VectorXd& times,
        Eigen::MatrixXd& muscleTendonVelocities) const {
    OPENSIM_THROW_IF(times[0] < getInitialTime(), Exception,
            "Requested initial time is lower than permitted.");
    OPENSIM_THROW_IF(times[times.size()-1] < getFinalTime(), Exception,
            "Requested final time is greater than permitted.");

    muscleTendonVelocities.resize(_numActiveMuscles, times.size());
    // The matrix is in column-major format.
    for (size_t i_mesh = 0; i_mesh < size_t(times.size()); ++i_mesh) {
        SimTK::Vector time(1, times[i_mesh]);
        for (size_t i_mus = 0; i_mus < _numActiveMuscles; ++i_mus) {
            // TODO should we use the spline derivative or what? it's helpful
            // when we don't have generalized speeds data.
            // The '{0}' means taking the first derivative w.r.t. time.
            muscleTendonVelocities(i_mus, i_mesh) =
                    _muscleTendonLengths.get(i_mus).calcDerivative({0}, time);
            // TODO muscleTendonVelocities(i_mus, i_mesh) =
            // TODO         _muscleTendonVelocities.get(i_mus).calcValue(time);
        }
    }
}

void InverseMuscleSolverMotionData::interpolateMomentArms(
        const Eigen::VectorXd& times,
        std::vector<Eigen::MatrixXd>& momentArms) const {
    OPENSIM_THROW_IF(times[0] < getInitialTime(), Exception,
            "Requested initial time is lower than permitted.");
    OPENSIM_THROW_IF(times[times.size()-1] < getFinalTime(), Exception,
            "Requested final time is greater than permitted.");

    momentArms.resize(times.size());
    for (size_t i_mesh = 0; i_mesh < size_t(times.size()); ++i_mesh) {
        momentArms[i_mesh].resize(_numCoordsToActuate, _numActiveMuscles);
        SimTK::Vector time(1, times[i_mesh]);
        // The matrix is in column-major format.
        for (size_t i_mus = 0; i_mus < _numActiveMuscles; ++i_mus) {
            for (size_t i_dof = 0; i_dof < _numCoordsToActuate; ++i_dof) {
                momentArms[i_mesh](i_dof, i_mus) =
                        _momentArms[i_dof].get(i_mus).calcValue(time);
            }
        }
    }
}

std::vector<std::string>
InverseMuscleSolverMotionData::createCoordPathsToActuate(const Model& model,
        const std::vector<const Coordinate*>& coordsToActuate) const {
    std::vector<std::string> coordPathsToActuate(coordsToActuate.size());
    const ComponentPath modelPath = model.getAbsolutePathName();
    for (size_t iCoord = 0; iCoord < coordsToActuate.size(); ++iCoord) {
        ComponentPath absPath(coordsToActuate[iCoord]->getAbsolutePathName());
        coordPathsToActuate[iCoord] =
                absPath.formRelativePath(modelPath).toString();
    }
    return coordPathsToActuate;
}

void InverseMuscleSolverMotionData::computeInverseDynamics(
        const OpenSim::Model& model,
        const std::vector<std::string>& coordPathsToActuate,
        const TimeSeriesTable& kinematicsData,
        const double& lowpassCutoffJointMoments) {

    Model modelForID(model);
    modelForID.finalizeFromProperties();
    // Disable all actuators in the model, as we don't want them to
    // contribute generalized forces that would reduce the inverse
    // dynamics moments to track.
    auto actuators = modelForID.updComponentList<Actuator>();
    for (auto& actuator : actuators) {
        actuator.set_appliesForce(false);
    }

    InverseDynamicsSolver invdyn(modelForID);
    SimTK::State state = modelForID.initSystem();

    // Assemble functions for coordinate values.
    // -----------------------------------------
    // Functions must be in the same order as the joint moments (multibody tree
    // order); that's why it's important that coordPathsToActuate is in
    // multibody tree order.
    // TODO uh oh separate list of coordinates.
    // To specify which columns of the kinematics table we want:
    std::vector<std::string> coordValLabels(coordPathsToActuate.size());
    for (size_t i = 0; i < coordValLabels.size(); ++i) {
        // This will yield something like "knee/flexion/value".
        coordValLabels[i] = coordPathsToActuate[i] + "/value";
    }

    // Indices of actuated coordinates in the list of all coordinates.
    // TODO do something correct here.
    std::vector<size_t> coordActIndices;
    auto coordsInOrder = modelForID.getCoordinatesInMultibodyTreeOrder();
    const ComponentPath modelPath = model.getAbsolutePathName();
    for (const auto& coordPathAct : coordPathsToActuate) {
        // Find the index of the coordinate with path coordPathAct.
        size_t iCoordAct = 0;
        for (auto& coord : coordsInOrder) {
            const auto thisCoordPath =
                    ComponentPath(coord->getAbsolutePathName())
                            .formRelativePath(modelPath).toString();
            if (coordPathAct == thisCoordPath) {
                coordActIndices.push_back(iCoordAct);
                break;
            }
            ++iCoordAct;
            // The first state variable in the coordinate should be its value.
            // TODOosim make it easy to get the full name of a state variable
            // Perhaps expose the StateVariable class.
            //coordValLabels[i] = coords[i]->getStateVariableNames()[0];
            // This will yield something like "knee/flexion/value".
            //coordActPaths.push_back(ComponentPath
            //// (coords[i]->getAbsolutePathName())//
            //        .formRelativePath(modelForID.// getAbsolutePathName()).toString());
            //coordValLabels.push_back(coordActPaths.back() + "/value");
        }
    }
    OPENSIM_THROW_IF(coordActIndices.size() != coordPathsToActuate.size(),
            Exception, "Internal bug detected: inconsistent number of "
            "actuated coordinates.");
    auto coordFunctions = createGCVSplineSet(kinematicsData, coordValLabels);

    // For debugging, print the splined coordinates, speeds, and accelerations.
    // auto coordSto = std::unique_ptr<Storage>(
    //         coordFunctions.constructStorage(0));
    // coordSto->print("DEBUG_splinedCoordinates.sto");
    // auto speedSto = std::unique_ptr<Storage>(
    //         coordFunctions.constructStorage(1));
    // speedSto->print("DEBUG_splinedSpeeds.sto");
    // auto accelSto = std::unique_ptr<Storage>(
    //         coordFunctions.constructStorage(2));
    // accelSto->print("DEBUG_splinedAccelerations.sto");

    // Convert normalized tropter points into times at which to evaluate
    // net joint moments.
    // TODO this variant ignores our data for generalized speeds.
    const auto& times = kinematicsData.getIndependentColumn();
    SimTK::Array_<double> simtkTimes(times);

    /* For debugging: use exact inverse dynamics solution.
    auto table = CSVFileAdapter::read(
            "DEBUG_testTugOfWar_INDYGO_actualInvDyn.csv");
    _netGeneralizedForces = createGCVSplineSet(table);
     */

    // Perform Inverse Dynamics.
    // -------------------------
    SimTK::Array_<SimTK::Vector> forceTrajectory;
    invdyn.solve(state, coordFunctions, simtkTimes, forceTrajectory);
    if (!forceTrajectory.empty()) {
        // If this exception is ever thrown, then it means we made the wrong
        // assumption constrained coordinates do not have an entry in
        // forceTrajectory.
        OPENSIM_THROW_IF(forceTrajectory[0].size() != coordFunctions.getSize(),
                Exception, "Internal bug detected: unexpected number of net "
                "generalized forces.");
    }

    // Post-process Inverse Dynamics results.
    // --------------------------------------
    Storage forceTrajectorySto;
    // TODO only store the columns corresponding to coordsToActuate.
    OpenSim::Array<std::string> labels("", coordPathsToActuate.size() + 1);
    labels[0] = "time";
    for (size_t i = 0; i < coordPathsToActuate.size(); ++i) {
        labels[i + 1] = coordPathsToActuate[i];
    }
    forceTrajectorySto.setColumnLabels(labels);
    SimTK::Vector row(_numCoordsToActuate);
    for (size_t i_time = 0; i_time < forceTrajectory.size(); ++i_time) {
        for (size_t i_coord = 0; i_coord < _numCoordsToActuate; ++i_coord) {
            row[i_coord] = forceTrajectory[i_time][coordActIndices[i_coord]];
        }
        forceTrajectorySto.append(times[i_time], row);
    }
    // forceTrajectorySto.print("DEBUG_desiredMoments_unfiltered.sto");
    const double& cutoffFrequency = lowpassCutoffJointMoments;
    if (cutoffFrequency > 0) {
        // Filter; otherwise, inverse dynamics moments are too noisy.
        forceTrajectorySto.pad(forceTrajectorySto.getSize()/2);
        forceTrajectorySto.lowpassIIR(cutoffFrequency);
    }
    _netGeneralizedForces = GCVSplineSet(5, &forceTrajectorySto);
}
