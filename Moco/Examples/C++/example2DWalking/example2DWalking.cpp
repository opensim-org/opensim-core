/* -------------------------------------------------------------------------- *
 * OpenSim Moco: example2DWalking.cpp                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
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

#include <Moco/osimMoco.h>

using namespace OpenSim;

// MocoGoal imposing an average gait speed through endpoint constraints. The
// average gait speed is defined as the distance traveled by the pelvis in the
// forward direction divided by the final time.
class MocoGaitSpeedGoal : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoGaitSpeedGoal, MocoGoal);
public:
    OpenSim_DECLARE_PROPERTY(gait_speed, double,
            "The average gait speed defined as the distance traveled by "
            "the pelvis in the forward direction divided by the final time.");
    MocoGaitSpeedGoal() {
        constructProperties();
    }
    MocoGaitSpeedGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
protected:
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::EndpointConstraint;
    }
    void calcGoalImpl(const GoalInput& input, SimTK::Vector& values)
        const override {
        // get final time
        SimTK::Real timeFinal = input.final_state.getTime();
        // get initial and final pelvis forward coordinate values
        SimTK::Real pelvisTxInitial =  m_coord->getValue(input.initial_state);
        SimTK::Real pelvisTxFinal =  m_coord->getValue(input.final_state);
        // calculate distance traveled
        SimTK::Real distanceTraveled = pelvisTxFinal - pelvisTxInitial;
        // calculate average gait speed
        values[0] = get_gait_speed() - (distanceTraveled / timeFinal);
    }
    void initializeOnModelImpl(const Model& model) const override {
        m_coord.reset(&model.getCoordinateSet().get("pelvis_tx"));
        setNumIntegralsAndOutputs(0, 1);
    }
private:
    void constructProperties() {
        constructProperty_gait_speed(0);
    }
    mutable SimTK::ReferencePtr<const Coordinate> m_coord;
};

// MocoGoal minimizing the integral of the squared controls divided by the
// distance traveled by the pelvis in the forward direction.
class MocoEffortOverDistanceGoal : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoEffortOverDistanceGoal, MocoGoal);
public:
    MocoEffortOverDistanceGoal() = default;
    MocoEffortOverDistanceGoal(std::string name)
            : MocoGoal(std::move(name)) {}
    MocoEffortOverDistanceGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {}
protected:
    void calcGoalImpl(const GoalInput& input, SimTK::Vector& goal)
        const override {
        // get initial and final pelvis forward coordinate values
        SimTK::Real pelvisTxInitial =  m_coord->getValue(input.initial_state);
        SimTK::Real pelvisTxFinal =  m_coord->getValue(input.final_state);
        // calculate distance traveled
        SimTK::Real distanceTraveled = pelvisTxFinal - pelvisTxInitial;
        // normalize integral by distance traveled
        goal[0] = input.integral / distanceTraveled ;
    }
    void calcIntegrandImpl(
        const SimTK::State& state, double& integrand) const override {
        // integrand is cubed controls
        const auto& controls = getModel().getControls(state);
        integrand = 0;
        for (int i = 0; i < getModel().getNumControls(); ++i)
            integrand += SimTK::cube(abs(controls[i]));
    }
    void initializeOnModelImpl(const Model& model) const override {
        m_coord.reset(&model.getCoordinateSet().get("pelvis_tx"));
        setNumIntegralsAndOutputs(1, 1);
    }
private:
    mutable SimTK::ReferencePtr<const Coordinate> m_coord;
};

// Set a coordinate tracking problem where the goal is to minimize the
// difference between provided and simulated coordinate values and speeds
// as well as to minimize an effort cost (squared controls). The provided data
// represents half a gait cycle. Endpoint constraints enforce periodicity of
// the coordinate values (except for pelvis tx) and speeds, coordinate
// actuator controls, and muscle activations. The tracking problem is solved
// using polynomial approximations of muscle path lengths if true is passed as
// input argument, whereas geometry paths are used with the argument false.
// Polynomial approximations should improve the computation speeds by about 15%
// for this problem.
MocoSolution gaitTracking(const bool& setPathLengthApproximation) {

    MocoTrack track;
    track.setName("gaitTracking");

    // Define the optimal control problem.
    // ===================================
    ModelProcessor modelprocessor = ModelProcessor("gait10dof18musc.osim") |
            ModOpSetPathLengthApproximation(setPathLengthApproximation);
    track.setModel(modelprocessor);
    track.setStatesReference(TableProcessor("referenceCoordinates.sto") |
            TabOpLowPassFilter(6));
    track.set_states_global_tracking_weight(10.0);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    track.set_apply_tracked_states_to_guess(true);
    track.set_initial_time(0.0);
    track.set_final_time(0.47008941);
    MocoStudy moco = track.initialize();
    MocoProblem& problem = moco.updProblem();

    // Goals.
    // =====
    // Symmetry
    auto* symmetryGoal = problem.addGoal<MocoPeriodicityGoal>("symmetryGoal");
    // Symmetric coordinate values
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tilt/value"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_ty/value"});
    symmetryGoal->addStatePair({"/jointset/hip_l/hip_flexion_l/value",
            "/jointset/hip_r/hip_flexion_r/value"});
    symmetryGoal->addStatePair({"/jointset/hip_r/hip_flexion_r/value",
            "/jointset/hip_l/hip_flexion_l/value"});
    symmetryGoal->addStatePair({"/jointset/knee_l/knee_angle_l/value",
            "/jointset/knee_r/knee_angle_r/value"});
    symmetryGoal->addStatePair({"/jointset/knee_r/knee_angle_r/value",
            "/jointset/knee_l/knee_angle_l/value"});
    symmetryGoal->addStatePair({"/jointset/ankle_l/ankle_angle_l/value",
            "/jointset/ankle_r/ankle_angle_r/value"});
    symmetryGoal->addStatePair({"/jointset/ankle_r/ankle_angle_r/value",
            "/jointset/ankle_l/ankle_angle_l/value"});
    symmetryGoal->addStatePair({"/jointset/lumbar/lumbar/value"});
    // Symmetric coordinate speeds
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tilt/speed"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tx/speed"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_ty/speed"});
    symmetryGoal->addStatePair({"/jointset/hip_l/hip_flexion_l/speed",
            "/jointset/hip_r/hip_flexion_r/speed"});
    symmetryGoal->addStatePair({"/jointset/hip_r/hip_flexion_r/speed",
            "/jointset/hip_l/hip_flexion_l/speed"});
    symmetryGoal->addStatePair({"/jointset/knee_l/knee_angle_l/speed",
            "/jointset/knee_r/knee_angle_r/speed"});
    symmetryGoal->addStatePair({"/jointset/knee_r/knee_angle_r/speed",
            "/jointset/knee_l/knee_angle_l/speed"});
    symmetryGoal->addStatePair({"/jointset/ankle_l/ankle_angle_l/speed",
            "/jointset/ankle_r/ankle_angle_r/speed"});
    symmetryGoal->addStatePair({"/jointset/ankle_r/ankle_angle_r/speed",
            "/jointset/ankle_l/ankle_angle_l/speed"});
    symmetryGoal->addStatePair({"/jointset/lumbar/lumbar/speed"});
    // Symmetric coordinate actuator controls
    symmetryGoal->addControlPair({"/lumbarAct"});
    // Symmetric muscle activations
    symmetryGoal->addStatePair({"/hamstrings_l/activation",
            "/hamstrings_r/activation"});
    symmetryGoal->addStatePair({"/hamstrings_r/activation",
            "/hamstrings_l/activation"});
    symmetryGoal->addStatePair({"/bifemsh_l/activation",
            "/bifemsh_r/activation"});
    symmetryGoal->addStatePair({"/bifemsh_r/activation",
            "/bifemsh_l/activation"});
    symmetryGoal->addStatePair({"/glut_max_l/activation",
            "/glut_max_r/activation"});
    symmetryGoal->addStatePair({"/glut_max_r/activation",
            "/glut_max_l/activation"});
    symmetryGoal->addStatePair({"/iliopsoas_l/activation",
            "/iliopsoas_r/activation"});
    symmetryGoal->addStatePair({"/iliopsoas_r/activation",
            "/iliopsoas_l/activation"});
    symmetryGoal->addStatePair({"/rect_fem_l/activation",
            "/rect_fem_r/activation"});
    symmetryGoal->addStatePair({"/rect_fem_r/activation",
            "/rect_fem_l/activation"});
    symmetryGoal->addStatePair({"/vasti_l/activation",
            "/vasti_r/activation"});
    symmetryGoal->addStatePair({"/vasti_r/activation",
            "/vasti_l/activation"});
    symmetryGoal->addStatePair({"/gastroc_l/activation",
            "/gastroc_r/activation"});
    symmetryGoal->addStatePair({"/gastroc_r/activation",
            "/gastroc_l/activation"});
    symmetryGoal->addStatePair({"/soleus_l/activation",
            "/soleus_r/activation"});
    symmetryGoal->addStatePair({"/soleus_r/activation",
            "/soleus_l/activation"});
    symmetryGoal->addStatePair({"/tib_ant_l/activation",
            "/tib_ant_r/activation"});
    symmetryGoal->addStatePair({"/tib_ant_r/activation",
            "/tib_ant_l/activation"});
    // Effort. Get a reference to the MocoControlGoal that is added to every
    // MocoTrack problem by default.
    MocoControlGoal& effort =
        dynamic_cast<MocoControlGoal&>(problem.updGoal("control_effort"));
    effort.setWeight(10);

    // Bounds.
    // =======
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
            {-20*SimTK::Pi/180, -10*SimTK::Pi/180});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0, 1});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
            {0.75, 1.25});
    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
            {-10*SimTK::Pi/180, 60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
            {-10*SimTK::Pi/180, 60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
            {-50*SimTK::Pi/180, 0});
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
            {-50*SimTK::Pi/180, 0});
    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
            {-15*SimTK::Pi/180, 25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
            {-15*SimTK::Pi/180, 25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/lumbar/lumbar/value",
            {0, 20*SimTK::Pi/180});

    // Configure the solver.
    // =====================
    MocoCasADiSolver& solver = moco.updSolver<MocoCasADiSolver>();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(1000);

    // Solve problem.
    // ==============
    MocoSolution solution = moco.solve();

    //moco.visualize(solution);

    return solution;
}

// Set a gait prediction problem where the goal is to minimize effort (squared
// controls) over distance traveled while enforcing symmetry of the walking
// cycle and a prescribed average gait speed through endpoint constraints. The
// solution of the coordinate tracking problem is passed as input argument and
// used as initial guess for the prediction. The predictive problem is solved
// using polynomial approximations of muscle path lengths if true is passed as
// input argument, whereas geometry paths are used with the argument false.
// Polynomial approximations should improve the computation speeds by about 25%
// for this problem.
void gaitPrediction(const MocoSolution& gaitTrackingSolution,
        const bool& setPathLengthApproximation){

    MocoStudy moco;
    moco.setName("gaitPrediction");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = moco.updProblem();
    ModelProcessor modelprocessor = ModelProcessor("gait10dof18musc.osim") |
            ModOpSetPathLengthApproximation(setPathLengthApproximation);
    problem.setModelProcessor(modelprocessor);

    // Goals.
    // =====
    // Symmetry
    auto* symmetryGoal = problem.addGoal<MocoPeriodicityGoal>("symmetryGoal");
    // Symmetric coordinate values
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tilt/value"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_ty/value"});
    symmetryGoal->addStatePair({"/jointset/hip_l/hip_flexion_l/value",
            "/jointset/hip_r/hip_flexion_r/value"});
    symmetryGoal->addStatePair({"/jointset/hip_r/hip_flexion_r/value",
            "/jointset/hip_l/hip_flexion_l/value"});
    symmetryGoal->addStatePair({"/jointset/knee_l/knee_angle_l/value",
            "/jointset/knee_r/knee_angle_r/value"});
    symmetryGoal->addStatePair({"/jointset/knee_r/knee_angle_r/value",
            "/jointset/knee_l/knee_angle_l/value"});
    symmetryGoal->addStatePair({"/jointset/ankle_l/ankle_angle_l/value",
            "/jointset/ankle_r/ankle_angle_r/value"});
    symmetryGoal->addStatePair({"/jointset/ankle_r/ankle_angle_r/value",
            "/jointset/ankle_l/ankle_angle_l/value"});
    symmetryGoal->addStatePair({"/jointset/lumbar/lumbar/value"});
    // Symmetric coordinate speeds
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tilt/speed"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tx/speed"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_ty/speed"});
    symmetryGoal->addStatePair({"/jointset/hip_l/hip_flexion_l/speed",
            "/jointset/hip_r/hip_flexion_r/speed"});
    symmetryGoal->addStatePair({"/jointset/hip_r/hip_flexion_r/speed",
            "/jointset/hip_l/hip_flexion_l/speed"});
    symmetryGoal->addStatePair({"/jointset/knee_l/knee_angle_l/speed",
            "/jointset/knee_r/knee_angle_r/speed"});
    symmetryGoal->addStatePair({"/jointset/knee_r/knee_angle_r/speed",
            "/jointset/knee_l/knee_angle_l/speed"});
    symmetryGoal->addStatePair({"/jointset/ankle_l/ankle_angle_l/speed",
            "/jointset/ankle_r/ankle_angle_r/speed"});
    symmetryGoal->addStatePair({"/jointset/ankle_r/ankle_angle_r/speed",
            "/jointset/ankle_l/ankle_angle_l/speed"});
    symmetryGoal->addStatePair({"/jointset/lumbar/lumbar/speed"});
    // Symmetric coordinate actuator controls
    symmetryGoal->addControlPair({"/lumbarAct"});
    // Symmetric muscle activations
    symmetryGoal->addStatePair({"/hamstrings_l/activation",
            "/hamstrings_r/activation"});
    symmetryGoal->addStatePair({"/hamstrings_r/activation",
            "/hamstrings_l/activation"});
    symmetryGoal->addStatePair({"/bifemsh_l/activation",
            "/bifemsh_r/activation"});
    symmetryGoal->addStatePair({"/bifemsh_r/activation",
            "/bifemsh_l/activation"});
    symmetryGoal->addStatePair({"/glut_max_l/activation",
            "/glut_max_r/activation"});
    symmetryGoal->addStatePair({"/glut_max_r/activation",
            "/glut_max_l/activation"});
    symmetryGoal->addStatePair({"/iliopsoas_l/activation",
            "/iliopsoas_r/activation"});
    symmetryGoal->addStatePair({"/iliopsoas_r/activation",
            "/iliopsoas_l/activation"});
    symmetryGoal->addStatePair({"/rect_fem_l/activation",
            "/rect_fem_r/activation"});
    symmetryGoal->addStatePair({"/rect_fem_r/activation",
            "/rect_fem_l/activation"});
    symmetryGoal->addStatePair({"/vasti_l/activation",
            "/vasti_r/activation"});
    symmetryGoal->addStatePair({"/vasti_r/activation",
            "/vasti_l/activation"});
    symmetryGoal->addStatePair({"/gastroc_l/activation",
            "/gastroc_r/activation"});
    symmetryGoal->addStatePair({"/gastroc_r/activation",
            "/gastroc_l/activation"});
    symmetryGoal->addStatePair({"/soleus_l/activation",
            "/soleus_r/activation"});
    symmetryGoal->addStatePair({"/soleus_r/activation",
            "/soleus_l/activation"});
    symmetryGoal->addStatePair({"/tib_ant_l/activation",
            "/tib_ant_r/activation"});
    symmetryGoal->addStatePair({"/tib_ant_r/activation",
            "/tib_ant_l/activation"});
    // Prescribed average gait speed
    auto* speedGoal = problem.addGoal<MocoGaitSpeedGoal>("speedGoal");
    speedGoal->set_gait_speed(1.2);
    // Effort over distance
    auto* effortGoal =
        problem.addGoal<MocoEffortOverDistanceGoal>("effortGoal", 10);

    // Bounds.
    // =======
    problem.setTimeBounds(0, {0.4,0.6});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
            {-20*SimTK::Pi/180, -10*SimTK::Pi/180});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0, 1});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
            {0.75, 1.25});
    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
            {-10*SimTK::Pi/180, 60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
            {-10*SimTK::Pi/180, 60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
            {-50*SimTK::Pi/180, 0});
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
            {-50*SimTK::Pi/180, 0});
    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
            {-15*SimTK::Pi/180, 25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
            {-15*SimTK::Pi/180, 25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/lumbar/lumbar/value",
            {0, 20*SimTK::Pi/180});

    // Configure the solver.
    // =====================
    auto& solver = moco.initCasADiSolver();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(1000);
    // Use the solution from the tracking simulation as initial guess.
    solver.setGuess(gaitTrackingSolution);

    // Solve problem.
    // ==============
    MocoSolution solution = moco.solve();

    moco.visualize(solution);
}

int main() {
    try {
        // Use polynomial approximations of muscle path lengths (set false to use
        // GeometryPath).
        const MocoSolution gaitTrackingSolution = gaitTracking(true);
        gaitPrediction(gaitTrackingSolution, true);
    } catch (const std::exception& e) {
        std::cout << e.what() << std::endl;
    }
    return EXIT_SUCCESS;
}
