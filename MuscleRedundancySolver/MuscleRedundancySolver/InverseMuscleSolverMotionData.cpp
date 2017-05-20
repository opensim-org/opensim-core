
#include "InverseMuscleSolverMotionData.h"

#include <mesh.h>

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
        const TimeSeriesTable& kinematicsData) :
        _initialTime(kinematicsData.getIndependentColumn().front()),
        _finalTime(kinematicsData.getIndependentColumn().back())
{
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
    for (size_t i_time = 0; i_time < kinematicsData.getNumRows(); ++i_time) {
        const auto& time = kinematicsData.getIndependentColumn()[i_time];
        SimTK::Vector row = kinematicsData.getRowAtIndex(i_time).transpose();
        statesSto.append(time, row);
    }
    auto statesTraj = StatesTrajectory::createFromStatesStorage(model,
            statesSto,
            true, false);
    // TODO give an error if the states do not contain generalized speeds.

    // Compute muscle quantities and spline the data.
    // ----------------------------------------------
    TimeSeriesTable muscleTendonLengths;
    TimeSeriesTable muscleTendonVelocities;
    // TODO get list of muscles from MuscleRedundancySolver.
    const auto muscleList = model.getComponentList<Muscle>();
    std::vector<const Muscle*> activeMuscles;
    auto coords = model.getCoordinatesInMultibodyTreeOrder();
    _numDOFs = coords.size();
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
        muscleTendonVelocities.setColumnLabels(musclePathNames);
        SimTK::RowVector rowMTL(musclePathNames.size());
        SimTK::RowVector rowMTV(musclePathNames.size());
        for (size_t i_time = 0; i_time < statesTraj.getSize(); ++i_time) {
            const auto& state = statesTraj[i_time];
            model.realizeVelocity(state);

            // TODO handle disabled muscles. (This might be outdated now).
            int i_muscle = 0;
            for (const auto* muscle : activeMuscles) {
                rowMTL[i_muscle] = muscle->getLength(state);
                rowMTV[i_muscle] = muscle->getLengtheningSpeed(state);
                i_muscle++;
            }
            muscleTendonLengths.appendRow(state.getTime(), rowMTL);
            muscleTendonVelocities.appendRow(state.getTime(), rowMTV);
        }
        _muscleTendonLengths = createGCVSplineSet(muscleTendonLengths);
        // TODO Separately splining muscleTendonLengths and velocities might
        // lead to inconsistency.
        _muscleTendonVelocities = createGCVSplineSet(muscleTendonVelocities);

        // Moment arms.
        // ````````````
        // This member holds moment arms across time, DOFs, and muscles.
        _momentArms.resize(_numDOFs);
        // Working memory.
        SimTK::RowVector rowMA(musclePathNames.size());
        for (size_t i_dof = 0; i_dof < _numDOFs; ++i_dof) {
            TimeSeriesTable momentArmsThisDOF;
            momentArmsThisDOF.setColumnLabels(musclePathNames);
            for (size_t i_time = 0; i_time < statesTraj.getSize(); ++i_time) {
                const auto& state = statesTraj[i_time];
                int i_muscle = 0;
                for (const auto* muscle : activeMuscles) {
                    rowMA[i_muscle] = muscle->computeMomentArm(state,
                            const_cast<Coordinate&>(*coords[i_dof]));
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
        const OpenSim::Model& model,
        const OpenSim::TimeSeriesTable& kinematicsData,
        const double& lowpassCutoffJointMoments) :
        InverseMuscleSolverMotionData(model, kinematicsData) {
    // Inverse dynamics.
    computeInverseDynamics(model, kinematicsData, lowpassCutoffJointMoments);
}

InverseMuscleSolverMotionData::InverseMuscleSolverMotionData(
        const OpenSim::Model& model,
        const TimeSeriesTable& kinematicsData,
        const TimeSeriesTable& netGeneralizedForcesData) :
        InverseMuscleSolverMotionData(model, kinematicsData) {
    // Inverse dynamics.
    // TODO validate column labels.
    _netGeneralizedForces = createGCVSplineSet(netGeneralizedForcesData);
}

void InverseMuscleSolverMotionData::interpolateNetGeneralizedForces(
        const Eigen::VectorXd& times,
        Eigen::MatrixXd& desiredMoments) const {
    OPENSIM_THROW_IF(times[0] < getInitialTime(), Exception,
                     "Initial time starts before the kinematics data.");
    OPENSIM_THROW_IF(times[times.size()-1] < getFinalTime(), Exception,
                     "Final time is beyond the end of the kinematics data.");

    desiredMoments.resize(_netGeneralizedForces.getSize(), times.size());
    for (size_t i_time = 0; i_time < size_t(times.size()); ++i_time) {
        for (size_t i_dof = 0; i_dof < size_t(_netGeneralizedForces.getSize());
             ++i_dof) {
            const double value = _netGeneralizedForces[i_dof].calcValue(
                    SimTK::Vector(1, times[i_time]));
            desiredMoments(i_dof, i_time) = value;
        }
    }
    mesh::write(times, desiredMoments, "DEBUG_desiredMoments.csv");
}

void InverseMuscleSolverMotionData::interpolateMuscleTendonLengths(
        const Eigen::VectorXd& times,
        Eigen::MatrixXd& muscleTendonLengths) const {
    OPENSIM_THROW_IF(times[0] < getInitialTime(), Exception,
                     "Initial time starts before the kinematics data.");
    OPENSIM_THROW_IF(times[times.size()-1] < getFinalTime(), Exception,
                     "Final time is beyond the end of the kinematics data.");

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
                     "Initial time starts before the kinematics data.");
    OPENSIM_THROW_IF(times[times.size()-1] < getFinalTime(), Exception,
                     "Final time is beyond the end of the kinematics data.");

    muscleTendonVelocities.resize(_numActiveMuscles, times.size());
    // The matrix is in column-major format.
    for (size_t i_mesh = 0; i_mesh < size_t(times.size()); ++i_mesh) {
        SimTK::Vector time(1, times[i_mesh]);
        for (size_t i_mus = 0; i_mus < _numActiveMuscles; ++i_mus) {
            muscleTendonVelocities(i_mus, i_mesh) =
                    _muscleTendonVelocities.get(i_mus).calcValue(time);
        }
    }
}

void InverseMuscleSolverMotionData::interpolateMomentArms(
        const Eigen::VectorXd& times,
        std::vector<Eigen::MatrixXd>& momentArms) const {
    OPENSIM_THROW_IF(times[0] < getInitialTime(), Exception,
                     "Initial time starts before the kinematics data.");
    OPENSIM_THROW_IF(times[times.size()-1] < getFinalTime(), Exception,
                     "Final time is beyond the end of the kinematics data.");

    momentArms.resize(times.size());
    for (size_t i_mesh = 0; i_mesh < size_t(times.size()); ++i_mesh) {
        momentArms[i_mesh].resize(_numDOFs, _numActiveMuscles);
        SimTK::Vector time(1, times[i_mesh]);
        // The matrix is in column-major format.
        for (size_t i_mus = 0; i_mus < _numActiveMuscles; ++i_mus) {
            for (size_t i_dof = 0; i_dof < _numDOFs; ++i_dof) {
                momentArms[i_mesh](i_dof, i_mus) =
                        _momentArms[i_dof].get(i_mus).calcValue(time);
            }
        }
    }
}

void InverseMuscleSolverMotionData::computeInverseDynamics(
        const OpenSim::Model& model,
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

    // Assemble functions for coordinate values. Functions must be in the
    // same order as the joint moments (multibody tree order).
    auto coords = modelForID.getCoordinatesInMultibodyTreeOrder();
    std::vector<std::string> columnLabels;
    for (size_t i = 0; i < coords.size(); ++i) {
        // We do not attempt to track constrained (e.g., locked) coordinates.
        // TODO create a test case for this.
        if (coords[i]->isConstrained(state)) continue;
        // The first state variable in the coordinate should be its value.
        // TODOosim make it easy to get the full name of a state variable
        // Perhaps expose the StateVariable class.
        //columnLabels[i] = coords[i]->getStateVariableNames()[0];
        // This will yield something like "knee/flexion/value".
        columnLabels.push_back(ComponentPath(coords[i]->getAbsolutePathName())
                .formRelativePath(modelForID.getAbsolutePathName()).toString()
                + "/value");
    }
    auto coordFunctions = createGCVSplineSet(kinematicsData, columnLabels);

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

    // Convert normalized mesh points into times at which to evaluate
    // net joint moments.
    // TODO this variant ignores our data for generalized speeds.
    const auto& times = kinematicsData.getIndependentColumn();
    // TODO avoid copy.
    SimTK::Array_<double> simtkTimes(times); // , SimTK::DontCopy());

    /* For debugging: use exact inverse dynamics solution.
    auto table = CSVFileAdapter::read(
            "DEBUG_testTugOfWar_MRS_actualInvDyn.csv");
    _netGeneralizedForces = createGCVSplineSet(table);
     */

    // Perform Inverse Dynamics.
    // -------------------------
    SimTK::Array_<SimTK::Vector> forceTrajectory;
    invdyn.solve(state, coordFunctions, simtkTimes, forceTrajectory);

    // Post-process Inverse Dynamics results.
    // --------------------------------------
    Storage forceTrajectorySto;
    const size_t numDOFs = forceTrajectory[0].size();
    OpenSim::Array<std::string> labels("", numDOFs + 1);
    labels[0] = "time";
    for (size_t i = 0; i < numDOFs; ++i) {
        labels[i + 1] = "force" + std::to_string(i);
    }
    forceTrajectorySto.setColumnLabels(labels);
    for (size_t i_time = 0; i_time < forceTrajectory.size(); ++i_time) {
        forceTrajectorySto.append(times[i_time], forceTrajectory[i_time]);
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
