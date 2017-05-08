#include <OpenSim/OpenSim.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>
#include <MuscleRedundancySolver.h>
#include <GlobalStaticOptimizationSolver.h>
#include <DeGroote2016Muscle.h>
#include <mesh.h>

#include "testing.h"

using namespace OpenSim;

// The objective of this test is to ensure that MRS functions properly with
// multiple muscles and degrees of freedom. TODO

// TODO now this is a minimum effort problem.

const double DISTANCE = 0.25;

// TODO type up documentation.
// TODO this problem has activation dynamics....should I remove that?
template <typename T>
class DeGroote2016MuscleTugOfWarMinEffortStatic
        : public mesh::OptimalControlProblemNamed<T> {
public:
    const double d = DISTANCE;
    double mass = -1;
    int m_i_position = -1;
    int m_i_speed = -1;
    int m_i_activation_l = -1;
    int m_i_activation_r = -1;
    int m_i_excitation_l = -1;
    int m_i_excitation_r = -1;
    DeGroote2016Muscle<T> m_muscleL;
    DeGroote2016Muscle<T> m_muscleR;

    DeGroote2016MuscleTugOfWarMinEffortStatic(const Model& model) :
            mesh::OptimalControlProblemNamed<T>("tug_of_war_min_effort") {
        this->set_time(0, 0.5);
        m_i_position =
                this->add_state("position", {-0.02, 0.02}, -0.015, 0.015);
        m_i_speed = this->add_state("speed", {-5, 15}, 0, 0);
        m_i_activation_l = this->add_state("activation_l", {0, 1}, 0);
        m_i_activation_r = this->add_state("activation_r", {0, 1}, 0);
        m_i_excitation_l = this->add_control("excitation_l", {0, 1});
        m_i_excitation_r = this->add_control("excitation_r", {0, 1});
        mass = dynamic_cast<const Body&>(model.getComponent("body")).get_mass();
        {
            const auto& osimMuscleL =
                    dynamic_cast<const Muscle&>(model.getComponent("left"));
            m_muscleL = DeGroote2016Muscle<T>(
                    osimMuscleL.get_max_isometric_force(),
                    osimMuscleL.get_optimal_fiber_length(),
                    osimMuscleL.get_tendon_slack_length(),
                    osimMuscleL.get_pennation_angle_at_optimal(),
                    osimMuscleL.get_max_contraction_velocity());
        }
        {
            const auto& osimMuscleR =
                    dynamic_cast<const Muscle&>(model.getComponent("right"));
            m_muscleR = DeGroote2016Muscle<T>(
                    osimMuscleR.get_max_isometric_force(),
                    osimMuscleR.get_optimal_fiber_length(),
                    osimMuscleR.get_tendon_slack_length(),
                    osimMuscleR.get_pennation_angle_at_optimal(),
                    osimMuscleR.get_max_contraction_velocity());
        }
    }
    void dynamics(const mesh::VectorX<T>& states,
                  const mesh::VectorX<T>& controls,
                  Eigen::Ref<mesh::VectorX<T>> derivatives) const override {
        // Unpack variables.
        // -----------------
        const T& position = states[m_i_position];
        const T& speed = states[m_i_speed];
        const T& activationL = states[m_i_activation_l];
        const T& activationR = states[m_i_activation_r];
        const T& excitationL = controls[m_i_excitation_l];
        const T& excitationR = controls[m_i_excitation_r];

        // Multibody kinematics.
        // ---------------------
        derivatives[m_i_position] = speed;

        // activation dynamics
        // fiber f-l
        // fiber f-v
        // passive f-l
        // tendon f-l

        T forceL = m_muscleL.calcRigidTendonFiberForceAlongTendon(
                    activationL, d + position, speed);
        T forceR = m_muscleR.calcRigidTendonFiberForceAlongTendon(
                    activationR, d - position, -speed);
        derivatives[m_i_speed] = (-forceL + forceR) / mass;

        // TODO remove activation dynamics.
        m_muscleL.calcActivationDynamics(excitationL, activationL,
                                         derivatives[m_i_activation_l]);
        m_muscleR.calcActivationDynamics(excitationR, activationR,
                                         derivatives[m_i_activation_r]);
    }
    void integral_cost(const T& /*time*/,
                       const mesh::VectorX<T>& /*states*/,
                       const mesh::VectorX<T>& controls,
                       T& integrand) const override {
        const auto& controlL = controls[m_i_excitation_l];
        const auto& controlR = controls[m_i_excitation_r];
        integrand = controlL * controlL + controlR * controlR;
    }
};


// TODO document.
template <typename T>
class DeGroote2016MuscleTugOfWarMinEffortDynamic
        : public mesh::OptimalControlProblemNamed<T> {
public:
    const double d = DISTANCE;
    double mass = -1;
    int m_i_position = -1;
    int m_i_speed = -1;
    int m_i_activation_l = -1;
    int m_i_activation_r = -1;
    int m_i_norm_fiber_length_l = -1;
    int m_i_norm_fiber_length_r = -1;
    int m_i_excitation_l = -1;
    int m_i_excitation_r = -1;
    int m_i_norm_fiber_velocity_l = -1;
    int m_i_fiber_equilibrium_l = -1;
    int m_i_norm_fiber_velocity_r = -1;
    int m_i_fiber_equilibrium_r = -1;

    DeGroote2016Muscle<T> m_muscle;

    DeGroote2016MuscleTugOfWarMinEffortDynamic(const Model& /*model*/) :
            mesh::OptimalControlProblemNamed<T>("tug_of_war_min_effort") {
        this->set_time(0, 0.5);
        m_i_position =
                this->add_state("position", {-0.02, 0.02}, /*TODO*/
                                -0.015, 0.015);
        m_i_speed = this->add_state("speed", {-5, 15}, 0, 0);
        m_i_activation_l = this->add_state("activation_l", {0, 1}, 0);
        m_i_activation_r = this->add_state("activation_r", {0, 1}, 0);
        m_i_norm_fiber_length_l = this->add_state("norm_fiber_length_l",
                                                  {0.2, 1.8});
        m_i_norm_fiber_length_r = this->add_state("norm_fiber_length_r",
                                                  {0.2, 1.8});
        m_i_excitation_l = this->add_control("excitation_l", {0, 1});
        m_i_excitation_r = this->add_control("excitation_r", {0, 1});
        m_i_norm_fiber_velocity_l = this->add_control("norm_fiber_velocity_l",
                                                      {-1, 1}, 0);
        m_i_norm_fiber_velocity_r = this->add_control("norm_fiber_velocity_r",
                                                      {-1, 1}, 0);
        m_i_fiber_equilibrium_l =
                this->add_path_constraint("fiber_equilibrium_l", 0);
        m_i_fiber_equilibrium_r =
                this->add_path_constraint("fiber_equilibrium_r", 0);
        mass = 1;
        m_muscle = DeGroote2016Muscle<adouble>(1, 1, 1, 1, 1);
    }
    void dynamics(const mesh::VectorX<T>& states,
                  const mesh::VectorX<T>& controls,
                  Eigen::Ref<mesh::VectorX<T>> derivatives) const override {
        // Unpack variables.
        // -----------------
        const T& position = states[m_i_position];
        const T& speed = states[m_i_speed];
        const T& activationL = states[m_i_activation_l];
        const T& activationR = states[m_i_activation_r];
        const T& normFibVelL = controls[m_i_norm_fiber_velocity_l];
        const T& normFibVelR = controls[m_i_norm_fiber_velocity_r];

        // Multibody kinematics.
        // ---------------------
        derivatives[m_i_position] = speed;

        const double max_force = 2;
        // activation dynamics
        // fiber f-l
        // fiber f-v
        // passive f-l
        // tendon f-l

        T forceL;
        {
            const T& normFibLenL = states[m_i_norm_fiber_length_l];
            // TODO pennation.
            const T normFiberForce = calcNormFiberForce(activationL,
                                                        normFibLenL,
                                                        normFibVelL);
            forceL = max_force * normFiberForce;
        }
        T forceR;
        {
            const T& normFibLenR = states[m_i_norm_fiber_length_r];
            // TODO pennation.
            const T normFiberForce = calcNormFiberForce(activationR,
                                                        normFibLenR,
                                                        normFibVelR);
            forceR = max_force * normFiberForce;
        }
        derivatives[m_i_speed] = (-forceL + forceR) / mass;

        const T& excitationL = controls[m_i_excitation_l];
        const T& excitationR = controls[m_i_excitation_r];
        derivatives[m_i_activation_l] = 1 / 0.05 * (excitationL - activationL);
        derivatives[m_i_activation_r] = 1 / 0.05 * (excitationR - activationR);

        const double max_contraction_velocity_l = 10;
        const double max_contraction_velocity_r = 10;
        derivatives[m_i_norm_fiber_length_l] = max_contraction_velocity_l *
                normFibVelL;
        derivatives[m_i_norm_fiber_length_r] = max_contraction_velocity_r *
                normFibVelR;
    }
    void path_constraints(unsigned /*i_mesh*/,
                          const T& /*time*/,
                          const mesh::VectorX<T>& states,
                          const mesh::VectorX<T>& controls,
                          Eigen::Ref<mesh::VectorX<T>> constraints)
    const override {
        const T& position = states[m_i_position];

        {
            const T& activationL = states[m_i_activation_l];
            const T& normFibLenL = states[m_i_norm_fiber_length_l];
            const T& normFibVelL = controls[m_i_norm_fiber_velocity_l];
            const T normFiberForce = calcNormFiberForce(activationL,
                                                        normFibLenL,
                                                        normFibVelL);
            const T tendon_slack_length_l = 0.20;
            const T optimal_fiber_length_l = 0.05;
            // TODO pennation.
            const T tendonLengthL = d + position
                    - optimal_fiber_length_l * normFibLenL;
            const T normTendonLengthL = tendonLengthL / tendon_slack_length_l;
            const T normTendonForce =
                    m_muscle.calcNormTendonForce(normTendonLengthL);
            constraints[m_i_fiber_equilibrium_l] =
                    normFiberForce - normTendonForce;
        }
        {
            const T& activationR = states[m_i_activation_r];
            const T& normFibLenR = states[m_i_norm_fiber_length_r];
            const T& normFibVelR = controls[m_i_norm_fiber_velocity_r];
            const T normFiberForce = calcNormFiberForce(activationR,
                                                        normFibLenR,
                                                        normFibVelR);

            // TODO make different from left muscle's properties.
            const T tendon_slack_length_r = 0.15;
            const T optimal_fiber_length_r = 0.10;
            // TODO pennation.
            const T tendonLengthR = d - position
                    - optimal_fiber_length_r * normFibLenR;
            const T normTendonLengthR = tendonLengthR / tendon_slack_length_r;
            const T normTendonForce =
                    m_muscle.calcNormTendonForce(normTendonLengthR);
            constraints[m_i_fiber_equilibrium_r] =
                    normFiberForce - normTendonForce;
        }
    }
    T calcNormFiberForce(const T& activation,
                         const T& normFibLen,
                         const T& normFibVel) const {
        const T forceLenMult =
                m_muscle.calcActiveForceLengthMultiplier(normFibLen);
        const T forceVelMult =
                m_muscle.calcForceVelocityMultiplier(normFibVel);
        const T normPassFibForce =
                m_muscle.calcNormPassiveFiberForce(normFibLen);
        return activation * forceLenMult * forceVelMult + normPassFibForce;
    }
    void integral_cost(const T& /*time*/,
                       const mesh::VectorX<T>& /*states*/,
                       const mesh::VectorX<T>& controls,
                       T& integrand) const override {
        const auto& controlL = controls[m_i_excitation_l];
        const auto& controlR = controls[m_i_excitation_r];
        integrand = controlL * controlL + controlR * controlR;
        //const auto& activationL = states[m_i_activation_l];
        //const auto& activationR = states[m_i_activation_r];
        //integrand = activationL * activationL + activationR * activationR;
    }
};

/// Two muscles coordinate to move a mass on a table (no gravity) from one
/// fixed state to another, in minimum time.
///
///                            mass
///                |------------O--------------|
///                   muscle L      muscle R
///
///                 <- - d- - ->|-> q
///
/// Here's a sketch of the problem we solve, with activation and fiber dynamics.
/// @verbatim
///   minimize   t_f
///   subject to qdot = u                  kinematics
///              udot = 1/m (-f_tL + f_tR) dynamics
///              aLdot = f_a(eL, aL)       activation dynamics
///              aRdot = f_a(eR, aR)
///              lmLdot = vmLdot           fiber dynamics
///              lmRdot = vmRdot
///              (a f_l(lm) f_v(vm) + f_p(lm)) cos(alpha) = f_t(lt) equilibrium
///              q(0) = -0.05
///              u(0) = 0
///              aL(0) = 0
///              aR(0) = 0
///              vmL(0) = 0
///              vmR(0) = 0
///              q(t_f) = 0.05
///              u(t_f) = 0
/// @endverbatim
/// This class can also solve the problem without muscle dynamics.

template <typename T>
class DeGroote2016MuscleTugOfWarMinTime
        : public mesh::OptimalControlProblemNamed<T> {
public:
    const double d = DISTANCE;
    double mass = -1;
    DeGroote2016Muscle<T> m_muscleL;
    DeGroote2016Muscle<T> m_muscleR;
    int m_i_position = -1;
    int m_i_speed = -1;
    int m_i_activation_l = -1;
    int m_i_norm_fiber_length_l = -1;
    int m_i_excitation_l = -1;
    int m_i_norm_fiber_velocity_l = -1;
    int m_i_fiber_equilibrium_l = -1;
    int m_i_activation_r = -1;
    int m_i_norm_fiber_length_r = -1;
    int m_i_excitation_r = -1;
    int m_i_norm_fiber_velocity_r = -1;
    int m_i_fiber_equilibrium_r = -1;

    DeGroote2016MuscleTugOfWarMinTime(const Model& model, bool muscleDynamics) :
            mesh::OptimalControlProblemNamed<T>("tug_of_war_min_time"),
            m_muscleDynamics(muscleDynamics) {
        this->set_time(0, {0.01, 0.5 /*TODO 0.3*/});
        m_i_position =
                this->add_state("position", {-0.02, 0.02}, /*TODO*/
                                -0.01, 0.01);
        m_i_speed = this->add_state("speed", {-5, 15}, 0, 0);
        if (muscleDynamics) {
            m_i_activation_l =
                    this->add_state("activation_l", {0, 1}/*TODO,  * 0*/);
            m_i_norm_fiber_length_l =
                    this->add_state("norm_fiber_length_l", {0.2, 1.8});
            m_i_excitation_l =
                    this->add_control("excitation_l", {0, 1});
            m_i_norm_fiber_velocity_l =
                    this->add_control("norm_fiber_velocity_l", {-1, 1});
            m_i_fiber_equilibrium_l =
                    this->add_path_constraint("fiber_equilibrium_l", 0);

            m_i_activation_r =
                    this->add_state("activation_r", {0, 1}/*, 0 TODO*/);
            m_i_norm_fiber_length_r =
                    this->add_state("norm_fiber_length_r", {0.2, 1.8});
            m_i_excitation_r =
                    this->add_control("excitation_r", {0, 1});
            m_i_norm_fiber_velocity_r =
                    this->add_control("norm_fiber_velocity_r", {-1, 1});/*TODO*/
            m_i_fiber_equilibrium_r =
                    this->add_path_constraint("fiber_equilibrium_r", 0);
        } else {
            m_i_activation_l = this->add_control("activation_l", {0, 1});
            m_i_activation_r = this->add_control("activation_r", {0, 1});
            m_i_excitation_l = m_i_activation_l;
            m_i_excitation_r = m_i_activation_r;
        }

        // Set parameters from the model.
        mass = model.getComponent<Body>("body").getMass();
        const auto& osimL = model.getComponent<Muscle>("left");
        m_muscleL =
                DeGroote2016Muscle<T>(osimL.get_max_isometric_force(),
                                      osimL.get_optimal_fiber_length(),
                                      osimL.get_tendon_slack_length(),
                                      osimL.get_pennation_angle_at_optimal(),
                                      osimL.get_max_contraction_velocity());
        const auto& osimR = model.getComponent<Muscle>("right");
        m_muscleR =
                DeGroote2016Muscle<T>(osimR.get_max_isometric_force(),
                                      osimR.get_optimal_fiber_length(),
                                      osimR.get_tendon_slack_length(),
                                      osimR.get_pennation_angle_at_optimal(),
                                      osimR.get_max_contraction_velocity());
    }
    void dynamics(const mesh::VectorX<T>& states,
                  const mesh::VectorX<T>& controls,
                  Eigen::Ref<mesh::VectorX<T>> derivatives) const override {
        // Unpack variables.
        // -----------------
        const T& position = states[m_i_position];
        const T& speed = states[m_i_speed];
        const T& activationL = m_muscleDynamics ?
                               states[m_i_activation_l] :
                               controls[m_i_activation_l];
        const T& activationR = m_muscleDynamics ?
                               states[m_i_activation_r] :
                               controls[m_i_activation_r];

        // Multibody kinematics.
        // ---------------------
        derivatives[m_i_position] = speed;

        // Multibody dynamics.
        // -------------------
        T forceL;
        T forceR;
        if (m_muscleDynamics) {
            const T& normFibLenL = states[m_i_norm_fiber_length_l];
            m_muscleL.calcTendonForce(d + position, normFibLenL, forceL);
            const T& normFibLenR = states[m_i_norm_fiber_length_r];
//            std::cout << "DEBUG d minus pos " <<
//                    static_cast<const double&>(d - position) << " " <<
//                    static_cast<const double&>(normFibLenR + 0)
//                    << std::endl;
            m_muscleR.calcTendonForce(d - position, normFibLenR, forceR);
            // forceL = activationL * m_muscleL.get_max_isometric_force();
            // forceR = activationR * m_muscleR.get_max_isometric_force();
            // std::cout << "DEBUG " << static_cast<const double&>(forceL) << " "
            //         <<
            //         static_cast<const double&>(forceR) <<
            //         std::endl;
            // TODO consider using fiber force instead; might depend more
            // directly on activation, etc; better conditioning?
            // TODO although it looks like optcntrlmuscle uses tendon force
            // also.

            // TODO make sure the muscle can be in equilibrium when buckling;
            // TODO could there be a scenario where the fiber is generating
            // passive force but the tendon is slack? TODO then the fiber
            // length is wrong!
        } else {
            forceL = m_muscleL.calcRigidTendonFiberForceAlongTendon(
                    activationL, d + position, speed);
            forceR = m_muscleR.calcRigidTendonFiberForceAlongTendon(
                    activationR, d - position, -speed);
        }

        derivatives[m_i_speed] = 1.0/mass * (-forceL + forceR);

        if (m_muscleDynamics) {
            // Activation dynamics.
            // --------------------
            {
                const T& excitationL = controls[m_i_excitation_l];
                m_muscleL.calcActivationDynamics(excitationL, activationL,
                                                 derivatives[m_i_activation_l]);
            }
            {
                const T& excitationR = controls[m_i_excitation_r];
                m_muscleR.calcActivationDynamics(excitationR, activationR,
                                                 derivatives[m_i_activation_r]);
            }

            // Fiber dynamics.
            // ---------------
            {
                const T& normFibVelL = controls[m_i_norm_fiber_velocity_l];
                derivatives[m_i_norm_fiber_length_l] =
                        m_muscleL.get_max_contraction_velocity() *
                        normFibVelL;
            }
            {
                const T& normFibVelR = controls[m_i_norm_fiber_velocity_l];
                derivatives[m_i_norm_fiber_length_r] =
                        m_muscleR.get_max_contraction_velocity() *
                        normFibVelR;
            }
        }
    }
    void path_constraints(unsigned /*i_mesh*/,
                          const T& /*time*/,
                          const mesh::VectorX<T>& states,
                          const mesh::VectorX<T>& controls,
                          Eigen::Ref<mesh::VectorX<T>> constraints)
    const override {
        if (m_muscleDynamics) {
            // Unpack variables.
            const T& position = states[m_i_position];

            // Left muscle.
            {
                const T& activationL = states[m_i_activation_l];
                const T& normFibLenL = states[m_i_norm_fiber_length_l];
                const T& normFibVelL = controls[m_i_norm_fiber_velocity_l];
                m_muscleL.calcEquilibriumResidual(
                        activationL, d + position, normFibLenL, normFibVelL,
                        constraints[m_i_fiber_equilibrium_l]);
            }
            // Right muscle.
            {
                const T& activationR = states[m_i_activation_r];
                const T& normFibLenR = states[m_i_norm_fiber_length_r];
                const T& normFibVelR = controls[m_i_norm_fiber_velocity_r];
                m_muscleR.calcEquilibriumResidual(
                        activationR, d - position, normFibLenR, normFibVelR,
                        constraints[m_i_fiber_equilibrium_r]);
            }
//            std::cout << "DEBUG FE " <<
//                    static_cast<const
//                    double&>(constraints[m_i_fiber_equilibrium_l]) <<
//                    " " <<
//                    static_cast<const
//                    double&>(constraints[m_i_fiber_equilibrium_r]) << std::endl;
        }
    }
    void integral_cost(const T& /*time*/,
                       const mesh::VectorX<T>& /*states*/,
                       const mesh::VectorX<T>& controls,
                       T& integrand) const override {
        const auto& controlL = controls[m_i_excitation_l];
        const auto& controlR = controls[m_i_excitation_r];
        integrand = controlL * controlL + controlR * controlR;
    }
    // void endpoint_cost(const T& final_time,
    //                    const mesh::VectorX<T>& /*final_states*/,
    //                    T& cost) const override {
    //     cost = final_time;
    // }
private:
    bool m_muscleDynamics;
};

OpenSim::Model buildTugOfWarModel() {
    using SimTK::Vec3;

    Model model;
    // model.setUseVisualizer(true);
    model.setName("tug_of_war");

    model.set_gravity(Vec3(0));
    auto* body = new Body("body", 1, Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint();
    joint->setName("joint");
    joint->connectSocket_parent_frame(model.getGround());
    joint->connectSocket_child_frame(*body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model.addComponent(joint);

    // The muscles' opt fib len + tendon slack length is slightly greater
    // than d.
    {
        auto* actuL = new Millard2012EquilibriumMuscle();
        actuL->setName("left");
        actuL->set_max_isometric_force(2);
        actuL->set_optimal_fiber_length(.20);
        actuL->set_tendon_slack_length(0.05);
        actuL->set_pennation_angle_at_optimal(0.05); // TODO
        // Attached the ground to the right of the pendulum.
        actuL->addNewPathPoint("origin", model.updGround(),
                               Vec3(-DISTANCE, 0, 0));
        // Attached to the mass at the end of the pendulum.
        actuL->addNewPathPoint("insertion", *body, Vec3(0));
        model.addComponent(actuL);
    }
    {
        auto* actuR = new Millard2012EquilibriumMuscle();
        actuR->setName("right");
        actuR->set_max_isometric_force(2);
        actuR->set_optimal_fiber_length(.15);
        actuR->set_tendon_slack_length(0.10);
        actuR->set_pennation_angle_at_optimal(0.01); // TODO
        // Attached the ground to the right of the pendulum.
        actuR->addNewPathPoint("origin", model.updGround(),
                               Vec3(DISTANCE, 0, 0));
        // Attached to the mass at the end of the pendulum.
        actuR->addNewPathPoint("insertion", *body, Vec3(0));
        model.addComponent(actuR);
    }

    // SimTK::State s = model.initSystem();
    // model.updMatterSubsystem().setShowDefaultGeometry(true);
    // model.updVisualizer().updSimbodyVisualizer().setBackgroundType(
    //         SimTK::Visualizer::GroundAndSky);
    // model.getVisualizer().show(s);
    // std::cin.get();
    // Manager manager(model);
    // manager.integrate(s, 1.0);
    return model;
}

template <typename T>
using TugOfWarStatic = DeGroote2016MuscleTugOfWarMinEffortStatic<T>;

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectoryGlobalStaticOptimizationSolver(const Model& model) {

    // Solve a trajectory optimization problem.
    // ----------------------------------------

    auto ocp = std::make_shared<TugOfWarStatic<adouble>>(model);
    ocp->print_description();
    const int N = 100;
    mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                                                  "ipopt", N);
    // Create an initial guess.
    // TODO using Eigen::RowVectorXd;
    // TODO mesh::OptimalControlIterate guess;
    // TODO guess.time.setLinSpaced(N, 0, 0.26);
    // TODO ocp->set_state_guess(guess, "angle",
    // TODO                      RowVectorXd::LinSpaced(N, 0, SimTK::Pi/4));
    // TODO ocp->set_state_guess(guess, "speed", RowVectorXd::LinSpaced(N, 6, 0));
    // TODO ocp->set_control_guess(guess, "activation",
    // TODO                        RowVectorXd::LinSpaced(N, 1.0, 0));

    // TODO mesh::OptimalControlSolution ocp_solution = dircol.solve(guess);
    mesh::OptimalControlSolution ocp_solution = dircol.solve();
    std::string trajectoryFile =
            "testTugOfWarDeGroote2016_GSO_trajectory.csv";
    ocp_solution.write(trajectoryFile);

    // Save the trajectory with a header so that OpenSim can read it.
    // --------------------------------------------------------------
    // CSVFileAdapter expects an "endheader" line in the file.
    auto fRead = std::ifstream(trajectoryFile);
    std::string trajFileWithHeader = trajectoryFile;
    trajFileWithHeader.replace(trajectoryFile.rfind(".csv"), 4,
                               "_with_header.csv");
    auto fWrite = std::ofstream(trajFileWithHeader);
    fWrite << "endheader" << std::endl;
    std::string line;
    while (std::getline(fRead, line)) fWrite << line << std::endl;
    fRead.close(); fWrite.close();

    // Create a table containing only the angle and speed of the pendulum.
    TimeSeriesTable ocpSolution = CSVFileAdapter::read(trajFileWithHeader);
    TimeSeriesTable kinematics;
    kinematics.setColumnLabels({"joint/position/value",
                                "joint/position/speed"});
    const auto& position = ocpSolution.getDependentColumn("position");
    const auto& speed = ocpSolution.getDependentColumn("speed");
    for (size_t iRow = 0; iRow < ocpSolution.getNumRows(); ++iRow) {
        SimTK::RowVector row(2);
        row[0] = position[iRow];
        row[1] = speed[iRow];
        kinematics.appendRow(ocpSolution.getIndependentColumn()[iRow], row);
    }

    // Compute actual inverse dynamics moment, for debugging.
    // ------------------------------------------------------
    TimeSeriesTable actualInvDyn;
    actualInvDyn.setColumnLabels({"inverse_dynamics"});
    auto ocpd = std::make_shared<TugOfWarStatic<double>>(model);
    for (Eigen::Index iTime = 0; iTime < ocp_solution.time.size(); ++iTime) {
        const auto& position = ocp_solution.states(0, iTime);
        const auto& speed = ocp_solution.states(1, iTime);
        const auto& activationL = ocp_solution.controls(0, iTime);
        const auto& activationR = ocp_solution.controls(1, iTime);
        auto forceL = ocpd->m_muscleL.calcRigidTendonFiberForceAlongTendon(
                activationL, DISTANCE + position, speed);;
        auto forceR = ocpd->m_muscleR.calcRigidTendonFiberForceAlongTendon(
                activationR, DISTANCE - position, -speed);
        actualInvDyn.appendRow(ocp_solution.time(iTime),
                               SimTK::RowVector(1, -forceL + forceR));
    }
    CSVFileAdapter::write(actualInvDyn,
        "DEBUG_testTugOfWar_GSO_actualInvDyn.csv");

    return {ocpSolution, kinematics};
}

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectoryMuscleRedundancySolver(const Model& model) {
    // Solve a trajectory optimization problem.
    // ----------------------------------------
    auto ocp = std::make_shared<DeGroote2016MuscleTugOfWarMinTime<adouble>>(
            model, true);
    ocp->print_description();
    const int N = 100;
    mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                                                  "ipopt", N);
    // TODO dircol.optimization_solver().set_max_iterations(1);
    // Create an initial guess.
    using Eigen::RowVectorXd;
    mesh::OptimalControlIterate guess;
    guess.time.setLinSpaced(N, 0, 0.3);
    ocp->set_state_guess(guess, "position",
                         RowVectorXd::LinSpaced(N, -0.01, 0.01));
    ocp->set_state_guess(guess, "speed", RowVectorXd::Constant(N, 2.0));
    ocp->set_state_guess(guess, "activation_l",
                         RowVectorXd::LinSpaced(N, 0, 1));
    ocp->set_state_guess(guess, "norm_fiber_length_l",
                         RowVectorXd::LinSpaced(N, 0.4, 1.25));
    ocp->set_control_guess(guess, "excitation_l",
                           RowVectorXd::LinSpaced(N, 0, 1));
    ocp->set_control_guess(guess, "norm_fiber_velocity_l",
                           RowVectorXd::Constant(N, 0.2));
    ocp->set_state_guess(guess, "activation_r",
                         RowVectorXd::LinSpaced(N, 1, 0));
    ocp->set_state_guess(guess, "norm_fiber_length_r",
                         RowVectorXd::LinSpaced(N, 1.25, 0.4));
    ocp->set_control_guess(guess, "excitation_r",
                           RowVectorXd::LinSpaced(N, 1, 0));
    ocp->set_control_guess(guess, "norm_fiber_velocity_r",
                           RowVectorXd::Constant(N, -0.2));

    mesh::OptimalControlSolution ocp_solution = dircol.solve(guess);
    dircol.print_constraint_values(ocp_solution);

//    mesh::OptimalControlSolution ocp_solution = dircol.solve();

    std::string trajectoryFile =
            "testTugOfWarDeGroote2016_MRS_trajectory.csv";
    ocp_solution.write(trajectoryFile);

    // Save the trajectory with a header so that OpenSim can read it.
    // --------------------------------------------------------------
    // CSVFileAdapter expects an "endheader" line in the file.
    auto fRead = std::ifstream(trajectoryFile);
    std::string trajFileWithHeader = trajectoryFile;
    trajFileWithHeader.replace(trajectoryFile.rfind(".csv"), 4,
                               "_with_header.csv");
    auto fWrite = std::ofstream(trajFileWithHeader);
    fWrite << "endheader" << std::endl;
    std::string line;
    while (std::getline(fRead, line)) fWrite << line << std::endl;
    fRead.close();
    fWrite.close();

    // Create a table containing only the angle and speed of the pendulum.
    TimeSeriesTable ocpSolution = CSVFileAdapter::read(trajFileWithHeader);
    TimeSeriesTable kinematics;
    kinematics.setColumnLabels({"joint/position/value",
                                "joint/position/speed"});
    const auto& position = ocpSolution.getDependentColumn("position");
    const auto& speed = ocpSolution.getDependentColumn("speed");
    for (size_t iRow = 0; iRow < ocpSolution.getNumRows(); ++iRow) {
        SimTK::RowVector row(2);
        row[0] = position[iRow];
        row[1] = speed[iRow];
        kinematics.appendRow(ocpSolution.getIndependentColumn()[iRow], row);
    }

    // Compute actual inverse dynamics moment, for debugging.
    // ------------------------------------------------------
    // TimeSeriesTable actualInvDyn;
    // actualInvDyn.setColumnLabels({"inverse_dynamics"});
    // DeGroote2016Muscle<double> muscle(ocp->max_isometric_force,
    //                                   ocp->optimal_fiber_length,
    //                                   ocp->tendon_slack_length,
    //                                   ocp->pennation_angle_at_optimal,
    //                                   ocp->max_contraction_velocity);
    // for (Eigen::Index iTime = 0; iTime < ocp_solution.time.size(); ++iTime) {
    //     const auto& angle = ocp_solution.states(0, iTime);
    //     const auto& normFibLen = ocp_solution.states(3, iTime);
    //     const auto intAngle = SimTK::Pi / 2 - angle;
    //     const auto musTenLen = ocp->d * sqrt(2 * (1 - cos(intAngle)));
    //     const auto momArm = -ocp->d * sin(intAngle) /
    //             sqrt(2 * (1 - cos(intAngle)));
    //     double tendonForce;
    //     muscle.calcTendonForce(musTenLen, normFibLen, tendonForce);
    //     actualInvDyn.appendRow(ocp_solution.time(iTime),
    //                            SimTK::RowVector(1, -momArm * tendonForce));
    // }
    // CSVFileAdapter::write(actualInvDyn,
    //     "DEBUG_testTugOfWar_MRS_actualInvDyn.csv");

    return {ocpSolution, kinematics};
}

// Reproduce the trajectory (generated without muscle dynamics) using static
// optimization.
void test2Muscles1DOFGlobalStaticOptimizationSolver(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data,
        const Model& model) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    // Create the MuscleRedundancySolver.
    // ----------------------------------
    GlobalStaticOptimizationSolver mrs;
    mrs.setModel(model);
    mrs.setKinematicsData(kinematics);
    mrs.set_lowpass_cutoff_frequency_for_joint_moments(150);
    double reserveOptimalForce = 0.001;
    mrs.set_create_reserve_actuators(reserveOptimalForce);
    GlobalStaticOptimizationSolver::Solution solution = mrs.solve();
    solution.write("testTugOfWarDeGroote2016_GSO");

    // Compare the solution to the initial trajectory optimization solution.
    // ---------------------------------------------------------------------
    // TODO rootMeanSquare(solution.activation, ocpSolution, "activation", 0
    // TODO // .03);//
    // TODO auto reserveForceRMS = reserveOptimalForce *//
    // TODO         solution.other_controls.getDependentColumnAtIndex(0).nor// mRMS();
    // TODO SimTK_TEST(reserveForceRMS < 0.02);
}

// Reproduce the trajectory (generated with muscle dynamics) using the
// MuscleRedundancySolver.
void test2Muscles1DOFMuscleRedundancySolver(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data,
        const Model& model) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    // Create the MuscleRedundancySolver.
    // ----------------------------------
    MuscleRedundancySolver mrs;
    mrs.setModel(model);
    mrs.setKinematicsData(kinematics);
    // Without filtering, the moments have high frequency content,
    // probably related to unfiltered generalized coordinates and getting
    // accelerations from a spline fit.
    // TODO mrs.set_lowpass_cutoff_frequency_for_joint_moments(80);
    mrs.set_create_reserve_actuators(0.001);
    MuscleRedundancySolver::Solution solution = mrs.solve();
    solution.write("testTugOfWarDeGroote2016_MRS");

    // Compare the solution to the initial trajectory optimization solution.
    // ---------------------------------------------------------------------
    compare(solution.activation, ocpSolution, "activation", 0.05);
    compare(solution.norm_fiber_length, ocpSolution, "norm_fiber_length",
            0.005);

    // We use a weaker check for the controls; they don't match as well.
    rootMeanSquare(solution.excitation, ocpSolution, "excitation", 0.08);
    rootMeanSquare(solution.norm_fiber_velocity, ocpSolution,
                   "norm_fiber_velocity", 0.04);
}

int main() {
    SimTK_START_TEST("testTugOfWarDeGroote2016");
//        {
//            Model mTODO;
//            using TugOfWarStatic =
//            DeGroote2016MuscleTugOfWarMinEffortStatic<adouble>;
//            auto ocp = std::make_shared<TugOfWarStatic>(mTODO);
//            ocp->print_description();
//            const int N = 100;
//            mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
//                                                          "ipopt", N);
//            mesh::OptimalControlSolution ocp_solution = dircol.solve();
//            dircol.print_constraint_values(ocp_solution);
//            std::string trajectoryFile =
//                    "testTugOfWarDeGroote2016Static_DEBUG.csv";
//            ocp_solution.write(trajectoryFile);
//        }
//        std::cout << "DYNAMIC" << std::endl;
//        {
//            Model mTODO;
//            using TugOfWarDynamic =
//            DeGroote2016MuscleTugOfWarMinEffortDynamic<adouble>;
//            auto ocp = std::make_shared<TugOfWarDynamic>(mTODO);
//            ocp->print_description();
//            const int N = 100; // TODO
//            mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
//                                                          "ipopt", N);
//            mesh::OptimalControlSolution ocp_solution = dircol.solve();
//            dircol.print_constraint_values(ocp_solution);
//            std::string trajectoryFile =
//                    "testTugOfWarDeGroote2016Dynamic_DEBUG.csv";
//            ocp_solution.write(trajectoryFile);
//
//        }
        Model model = buildTugOfWarModel();
        model.finalizeFromProperties();
        {
            auto gsoData =
                    solveForTrajectoryGlobalStaticOptimizationSolver(model);
            // TODO
//            SimTK_SUBTEST2(test2Muscles1DOFGlobalStaticOptimizationSolver,
//                           gsoData, model);
        }
        {
//            auto mrsData = solveForTrajectoryMuscleRedundancySolver(model);
//            // TODO
//            SimTK_SUBTEST2(test2Muscles1DOFMuscleRedundancySolver, mrsData,
//                           model);
        }
    SimTK_END_TEST();
}
