/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxCalibrateContact.cpp                                  *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco, Chris Dembia                                   *
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


#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <Moco/osimMoco.h>

#include <MocoSandboxShared.h>

#include <tropter/tropter.h>

using namespace OpenSim;
using Eigen::VectorXd;

template <typename T, typename TStiffness>
T contact_force(const TStiffness& stiffness, const T& y) {
    const double stiffness_fictitious = 1.0; // N/m
    const T ground_height = 0;
    // Positive if penetrated.
    const T depth = ground_height - y;
    const T depth_pos = fmax(0, depth);
    const T contact_normal_force = stiffness * depth_pos +
            stiffness_fictitious * depth;
    return contact_normal_force;
}

template<typename T>
class BouncingBallLinear : public tropter::Problem<T> {
public:
    static const double mass; // kg
    static const double stiffness; // N/m
    static const double g; // m/s^2
    BouncingBallLinear() {
        this->set_time(0, 1.25);
        this->add_state("y", {-1, 1}, 1);
        this->add_state("vy", {-10, 10}, 0);
    }
    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {
        const T& y = in.states[0];
        const T& vy = in.states[1];
        out.dynamics[0] = vy;
        const auto contact_normal_force = contact_force(this->stiffness, y);
        out.dynamics[1] = -g + (contact_normal_force) / mass;
    }
    static tropter::Solution run() {
        auto ocp = std::make_shared<BouncingBallLinear<T>>();
        const int N = 1000;
        tropter::DirectCollocationSolver<T> dircol(ocp, "trapezoidal", "ipopt",
                N);
        tropter::Solution solution = dircol.solve();
        //std::cout << "States: " << solution.states << std::endl;
        //solution.write("sandboxCalibrateContact_bouncing_ball_solution.csv");
        return solution;
    }
};
template <typename T>
const double BouncingBallLinear<T>::mass = 50.0; // kg
template <typename T>
const double BouncingBallLinear<T>::stiffness = 3180.0; // N/m
template <typename T>
const double BouncingBallLinear<T>::g = 9.81; // m/s^2

class BallCalibration : public tropter::optimization::Problem<double> {
public:
    BallCalibration(Eigen::VectorXd yTraj, Eigen::VectorXd contactForceTraj) :
            tropter::optimization::Problem<double>(1, 0),
            m_yTraj(yTraj), m_contactForceTraj(contactForceTraj) {
        Eigen::VectorXd lower(1); lower[0] = 0;
        Eigen::VectorXd upper(1); upper[0] = 10000;
        set_variable_bounds(lower, upper);
    }
    void calc_objective(const VectorXd& x, double& obj_value) const override {

        const auto& stiffness = x[0];
        obj_value = 0;
        for (int it = 0; it < m_yTraj.size(); ++it) {
            const auto& y = m_yTraj[it];
            const auto simContactForce = contact_force(stiffness, y);
            const auto& expContactForce = m_contactForceTraj[it];
            obj_value += pow(simContactForce - expContactForce, 2);
        }
    }
private:
    Eigen::VectorXd m_yTraj;
    Eigen::VectorXd m_contactForceTraj;
};

void calibrateBall() {
    const auto exp = BouncingBallLinear<adouble>::run();
    Eigen::VectorXd Fy(exp.time.size());
    for (int it = 0; it < exp.time.size(); ++it) {
        Fy[it] = contact_force(BouncingBallLinear<double>::stiffness,
                exp.states(0, it));
    }
    BallCalibration problem(exp.states.row(0).transpose(), Fy);
    tropter::optimization::IPOPTSolver solver(problem);
    solver.set_verbosity(1);
    auto solution = solver.optimize();
    std::cout << solution.variables << std::endl;
}

enum class ContactModel {
    AckermannVanDenBogert,
    HuntCrossley
};

/// Optimize the stiffness of contact points to match the ground reaction
/// force.
class ContactCalibration : public tropter::optimization::Problem<double> {
public:
    ContactCalibration(Model model, StatesTrajectory statesTraj,
            TimeSeriesTable grfData, std::string grfPrefix,
            ContactModel contactModel, int numContacts,
            bool optimizeIndividualMarkerHeights) :
            m_model(std::move(model)), m_statesTraj(std::move(statesTraj)),
            m_contactModel(contactModel), m_numContacts(numContacts),
            m_optimizeIndividualMarkerHeights(optimizeIndividualMarkerHeights) {

        int numVariables = -1;
        if (contactModel == ContactModel::AckermannVanDenBogert) {
            if (optimizeIndividualMarkerHeights) {
                numVariables = 5 * numContacts;
            } else {
                numVariables = 4 * numContacts + 1;
            }
        } else if (contactModel == ContactModel::HuntCrossley) {
            if (!optimizeIndividualMarkerHeights) {
                throw std::runtime_error("Hunt-Crossley requires optimizing "
                        "individual marker heights.");
            }
            numVariables = 6 * numContacts;
        }
        set_num_variables(numVariables);
        set_variable_bounds(Eigen::VectorXd::Zero(get_num_variables()),
                Eigen::VectorXd::Ones(get_num_variables()));

        m_model.initSystem();

        // Foot–ground contact data.
        // -------------------------
        auto time = grfData.getIndependentColumn();
        SimTK::Vector Fx = grfData.getDependentColumn(grfPrefix + "_vx");
        SimTK::Vector Fy = grfData.getDependentColumn(grfPrefix + "_vy");
        m_FxSpline = GCVSpline(5, (int)time.size(), time.data(), &Fx[0]);
        m_FySpline = GCVSpline(5, (int)time.size(), time.data(), &Fy[0]);
    }

    void applyParametersToModel(const VectorXd& x, Model& model) const {
        if (m_contactModel == ContactModel::AckermannVanDenBogert) {
            auto applyMarkerHeight = [&model](const std::string& name,
                    const double& normHeight) {
                auto& marker = model.updComponent<Marker>(name);
                // index 1 for y component.
                const double lower = -0.06;
                const double upper =  0.05;
                marker.upd_location()[1] = lower + normHeight * (upper - lower);
            };
            for (int icontact = 0; icontact < m_numContacts; ++icontact) {
                const std::string name = "marker" + std::to_string(icontact);
                if (m_optimizeIndividualMarkerHeights) {
                    applyMarkerHeight(name, x[icontact]);
                } else {
                    applyMarkerHeight(name, x[0]);
                }
            }

            double stiffnessScalingFactor = 1e8;
            double frictionCoeffScalingFactor = 1.0;
            double dissipationScalingFactor = 2.0;
            double velScalingMin = 0.01;
            double velScalingMax = 0.10;

            int icontact = 0;
            int offset = (m_optimizeIndividualMarkerHeights ? m_numContacts : 1);
            for (auto& contact :
                    model.updComponentList<AckermannVanDenBogert2010Force>()) {
                contact.set_stiffness(
                        stiffnessScalingFactor * x[icontact + offset]);
                contact.set_dissipation(
                        dissipationScalingFactor * x[icontact + 1 * m_numContacts + offset]);
                contact.set_friction_coefficient(
                        frictionCoeffScalingFactor * x[icontact + 2 * m_numContacts + offset]);
                contact.set_tangent_velocity_scaling_factor(
                        velScalingMin + x[icontact + 3 * m_numContacts + offset] *
                                (velScalingMax - velScalingMin));
                ++icontact;
            }
        } else if (m_contactModel == ContactModel::HuntCrossley) {
            for (int icontact = 0; icontact < m_numContacts; ++icontact) {
                auto& contactGeom = model.updComponent<ContactSphere>(
                        "contact" + std::to_string(icontact));
                {
                    const double lower = -0.06;
                    const double upper =  0.05;
                    const auto& normHeight = x[icontact];
                    // index 1 for y component.
                    contactGeom.upd_location()[1] = lower + normHeight * (upper - lower);
                }
                // TODO scaling x position of elements.

                {
                    const double lower = 0.01;
                    const double upper = 0.04;
                    const auto& normRadius = x[icontact + m_numContacts];
                    contactGeom.setRadius(
                            lower + normRadius * (upper - lower));
                }

                auto& contactForce = model.updComponent<HuntCrossleyForce>(
                        "contact" + std::to_string(icontact) + "_force");
                {
                    double scalingFactor = 1e8;
                    const auto& normStiffness = x[icontact + 2 * m_numContacts];
                    contactForce.setStiffness(scalingFactor * normStiffness);
                }
                {
                    const auto& normDissipation = x[icontact + 3 * m_numContacts];
                    contactForce.setDissipation(normDissipation);
                }
                {
                    const auto& frictionCoeff = x[icontact + 4 * m_numContacts];
                    contactForce.setStaticFriction(frictionCoeff);
                    contactForce.setDynamicFriction(frictionCoeff);
                }
                {
                    // TODO lower and upper are unused.
                    double lower = 0.01;
                    double upper = 0.10;
                    const auto& transitionVelocity =
                            x[icontact + 5 * m_numContacts];
                    contactForce.setTransitionVelocity(transitionVelocity);
                }
            }
        }
    }

    void calc_contact_force(const Model& model, SimTK::Matrix& forces) const {
        forces.resize((int)m_statesTraj.getSize(), 2);

        if (m_contactModel == ContactModel::AckermannVanDenBogert) {
            auto contacts =
                    model.getComponentList<AckermannVanDenBogert2010Force>();
            int itime = 0;
            for (auto state : m_statesTraj) {
                // Making a copy of the state (auto state instead of const auto&
                // state) is important for invalidating the cached contact point
                // locations.
                model.realizeVelocity(state);

                forces(itime, 0) = 0;
                forces(itime, 1) = 0;
                for (auto& contact : contacts) {
                    SimTK::Vec3 force = contact.calcContactForceOnStation(state);
                    forces(itime, 0) += force[0];
                    forces(itime, 1) += force[1];
                }
                ++itime;
            }
        } else if (m_contactModel == ContactModel::HuntCrossley) {
            auto contacts = model.getComponentList<HuntCrossleyForce>();
            int itime = 0;
            for (auto state : m_statesTraj) {
                model.realizeDynamics(state);
                forces(itime, 0) = 0;
                forces(itime, 1) = 0;
                for (auto& contact : contacts) {
                    // OpenSim::Array<std::string> recordLabels = contact.getRecordLabels();
                    // for (int i = 0; i < recordLabels.getSize(); ++i) {
                    //     std::cout << "DEBUGrlabels " << recordLabels[i] << std::endl;
                    // }
                    OpenSim::Array<double> record = contact.getRecordValues(state);
                    //  0: contacti_force.calcn.force.X
                    //  1: contacti_force.calcn.force.Y
                    //  2: contacti_force.calcn.force.Z
                    //  3: contacti_force.calcn.torque.X
                    //  4: contacti_force.calcn.torque.Y
                    //  5: contacti_force.calcn.torque.Z
                    //  6: contacti_force.ground.force.X
                    //  7: contacti_force.ground.force.Y
                    //  8: contacti_force.ground.force.Z
                    //  9: contacti_force.ground.torque.X
                    // 10: contacti_force.ground.torque.Y
                    // 11: contacti_force.ground.torque.Z
                    forces(itime, 0) += record[0];
                    forces(itime, 1) += record[1];
                }
                ++itime;
            }
        }
    }
    void calc_objective(const VectorXd& x, double& obj_value) const override {

        obj_value = 0;

        //std::cout << "DEBUGx " << x << std::endl;
        //std::cout << "DEBUG " << std::setprecision(16) <<
        //        x[0] << "\n" << x[1] << "\n" << x[2] << "\n" <<
        //        x[3] << "\n" << x[4] << "\n" << x[5] << std::endl;

        const auto thread_id = std::this_thread::get_id();
        if (m_workingModels.count(thread_id) == 0) {
            std::unique_lock<std::mutex> lock(m_modelMutex);
            m_workingModels[thread_id] = m_model;
            lock.unlock();
            m_workingModels[thread_id].initSystem();
        }
        if (m_workingForces.count(thread_id) == 0) {
            std::unique_lock<std::mutex> lock(m_forcesMutex);
            m_workingForces[thread_id] = SimTK::Matrix();
            lock.unlock();
        }
        Model& model = m_workingModels[thread_id];
        SimTK::Matrix& forces = m_workingForces[thread_id];

        // Apply parameters.
        // -----------------
        applyParametersToModel(x, model);
        // TODO is this expensive?
        model.initSystem();

        // Compute contact force error.
        // ----------------------------
        calc_contact_force(model, forces);
        for (int itime = 0; itime < (int)m_statesTraj.getSize(); ++itime) {
            const auto& simFx = forces(itime, 0);
            const auto& simFy = forces(itime, 1);
            SimTK::Vector timeVec(1, m_statesTraj[itime].getTime());
            SimTK::Real expFx = m_FxSpline.calcValue(timeVec);
            SimTK::Real expFy = m_FySpline.calcValue(timeVec);
            // Weight horizontal error more strongly.
            obj_value += 4 * pow(simFx - expFx, 2) + pow(simFy - expFy, 2);
        }
        const double mg = model.getTotalMass(m_statesTraj.front()) *
                model.getGravity().norm();
        // TODO how should we normalize?
        obj_value /= mg * m_statesTraj.getSize();

    }
    void printContactComparison(const VectorXd& x,
            const std::string& filename) {
        TimeSeriesTable table;

        if (m_contactModel == ContactModel::AckermannVanDenBogert) {
            for (int i = 0; i < x.size(); ++i) {
                int offset = (m_optimizeIndividualMarkerHeights ? m_numContacts : 1);
                if (i < offset)
                    std::cout << "marker height " << i;
                else if (i < 1 * m_numContacts + offset)
                    std::cout << "stiffness " << i - offset;
                else if (i < 2 * m_numContacts + offset)
                    std::cout << "dissipation " << i - 1 * m_numContacts - offset;
                else if (i < 3 * m_numContacts + offset)
                    std::cout << "coefficient of friction " << i - 2*m_numContacts - offset;
                else
                    std::cout << "transition velocity " << i - 3 * m_numContacts - offset;
                std::cout << ": " << x[i] << std::endl;
            }
        } else if (m_contactModel == ContactModel::HuntCrossley) {
            for (int i = 0; i < x.size(); ++i) {
                if (i < m_numContacts)
                    std::cout << "sphere height " << i;
                else if (i < 2 * m_numContacts)
                    std::cout << "radius " << i - m_numContacts;
                else if (i < 3 * m_numContacts)
                    std::cout << "stiffness " << i - 2 * m_numContacts;
                else if (i < 4 * m_numContacts)
                    std::cout << "dissipation " << i - 3 * m_numContacts;
                else if (i < 5 * m_numContacts)
                    std::cout << "coefficient of friction "
                            << i - 4 * m_numContacts;
                else if (i < 6 * m_numContacts)
                    std::cout << "transition velocity "
                            << i - 5 * m_numContacts;
                std::cout << ": " << x[i] << std::endl;
            }
        }

        // Apply parameters.
        // -----------------
        applyParametersToModel(x, m_model);
        // TODO is this expensive?
        m_model.initSystem();

        // Compute contact force error.
        // ----------------------------
        SimTK::Matrix forces;
        calc_contact_force(m_model, forces);

        // TODO add fore-aft force.

        table.setColumnLabels({"Fx_sim", "Fy_sim", "Fx_exp", "Fy_exp"});
        for (int itime = 0; itime < (int)m_statesTraj.getSize(); ++itime) {
            SimTK::RowVector row(4, 0.0);

            row[0] = forces(itime, 0);
            row[1] = forces(itime, 1);

            const auto& time = m_statesTraj[itime].getTime();
            SimTK::Vector timeVec(1, time);
            row[2] = m_FxSpline.calcValue(timeVec);
            row[3] = m_FySpline.calcValue(timeVec);

            table.appendRow(time, row);
        }
        STOFileAdapter::write(table, filename);
    }
private:

    mutable Model m_model;
    StatesTrajectory m_statesTraj;
    GCVSpline m_FxSpline;
    GCVSpline m_FySpline;
    ContactModel m_contactModel;
    int m_numContacts;
    bool m_optimizeIndividualMarkerHeights;

    mutable std::mutex m_modelMutex;
    mutable std::unordered_map<std::thread::id, Model> m_workingModels;
    mutable std::mutex m_forcesMutex;
    mutable std::unordered_map<std::thread::id, SimTK::Matrix> m_workingForces;

};

class SimTKContactCalibration : public SimTK::OptimizerSystem {
public:
    SimTKContactCalibration(Model model, StatesTrajectory statesTraj,
            TimeSeriesTable grfData, std::string grfPrefix,
            ContactModel contactModel, int numContacts,
            bool optimizeIndividualMarkerHeights)
            : SimTK::OptimizerSystem(),
              m_tropProb(std::move(model), std::move(statesTraj),
                      std::move(grfData), grfPrefix,
                      contactModel, numContacts,
                      optimizeIndividualMarkerHeights) {
        setNumParameters(m_tropProb.get_num_variables());
        int N = m_tropProb.get_num_variables();
        setParameterLimits(SimTK::Vector(N, 0.0), SimTK::Vector(N, 1.0));
    }
    void applyParametersToModel(const SimTK::Vector& vars, Model& model) const {
        Eigen::VectorXd x = Eigen::Map<const VectorXd>(&vars[0], vars.size());
        m_tropProb.applyParametersToModel(x, model);
    }
    int objectiveFunc(const SimTK::Vector& vars, bool, SimTK::Real& f)
    const override {
        ++m_objCount;
        Eigen::VectorXd x = Eigen::Map<const VectorXd>(&vars[0], vars.size());
        m_tropProb.calc_objective(x, f);
        std::cout << "DEBUG " << m_objCount << " " << f << " " << vars <<
                std::endl;
        return 0;
    }
    void printContactComparison(const SimTK::Vector& vars,
            const std::string& filename) {
        Eigen::VectorXd x = Eigen::Map<const VectorXd>(&vars[0], vars.size());
        m_tropProb.printContactComparison(x, filename);
    }
private:
    ContactCalibration m_tropProb;
    mutable std::atomic<int> m_objCount {0};
};

void addContact(Model& model, std::string markerName, double stiffness = 5e7,
        double dissipation = 1.0,
        double friction_coefficient = 0.95, double velocity_scaling = 0.3) {
    auto* contact = new AckermannVanDenBogert2010Force();
    contact->setName(markerName + "_contact");
    contact->set_stiffness(stiffness);
    contact->set_dissipation(dissipation);
    contact->set_friction_coefficient(friction_coefficient);
    contact->set_tangent_velocity_scaling_factor(velocity_scaling);
    model.addComponent(contact);
    contact->updSocket("station").setConnecteePath(markerName);
}

enum class DataType {
    Treadmill,
    Overground
};

void calibrateContact(DataType dataType, ContactModel contactModel,
        bool optimizeIndividualMarkerHeights = true) {

    // Model.
    // ------
    std::string modelFileName;
    TimeSeriesTable grfData;
    std::string grfPrefix;
    if (dataType == DataType::Treadmill) {
        modelFileName = "gait1018_subject01_onefoot_v30516.osim";
        grfData = STOFileAdapter::read("walk_gait1018_subject01_grf.mot");
        grfPrefix = "ground_force";
    } else if (dataType == DataType::Overground) {
        modelFileName = "Rajagopal2016_subject05_v30516.osim";
        grfData = STOFileAdapter::read("loadedwalking_subject05_noload_free_trial03_grf.mot");
        grfPrefix = "ground_force_r";
    }
    Model model(modelFileName);
    model.initSystem();

    const auto& calcn = dynamic_cast<Body&>(model.updComponent("calcn_r"));
    /*
    model.addMarker(new Marker("R.Heel.Distal", calcn,
            SimTK::Vec3(0.01548, -0.0272884, -0.00503735)));
    model.addMarker(new Marker("R.Ball.Lat", calcn,
            SimTK::Vec3(0.16769, -0.0272884, 0.066)));
    model.addMarker(new Marker("R.Ball.Med", calcn,
            SimTK::Vec3(0.1898, -0.0272884, -0.03237)));
    addContact(model, "R.Heel.Distal", 5e7);
    addContact(model, "R.Ball.Lat", 7.5e7);
    addContact(model, "R.Ball.Med", 7.5e7);
    */
    // Programmatically add contact points across the foot.
    int numContacts = -1;
    if (contactModel == ContactModel::AckermannVanDenBogert) {
        numContacts = 6;
        for (int icontact = 0; icontact < numContacts; ++icontact) {
            const std::string name = "marker" + std::to_string(icontact);
            const SimTK::Real xHeel = -0.02;
            SimTK::Real xToes = SimTK::NaN;
            if (dataType == DataType::Treadmill) {
                xToes =  0.27;
            } else if (dataType == DataType::Overground) {
                xToes =  0.29;
            }
            const SimTK::Real x = xHeel +
                    SimTK::Real(icontact) / SimTK::Real(numContacts - 1) *
                            (xToes - xHeel);
            model.addMarker(
                    new Marker(name, calcn, SimTK::Vec3(x, -0.027, 0.0)));
            addContact(model, name);
        }
    } else if (contactModel == ContactModel::HuntCrossley) {
        using SimTK::Vec3;
        auto* groundContact = new ContactHalfSpace(Vec3(0),
                Vec3(0, 0, -0.5 * SimTK::Pi),
                model.getGround(), "ground_halfspace");
        model.addContactGeometry(groundContact);

        numContacts = 5;
        // TODO make these parameters:
        const SimTK::Real xHeel =  0.00;
        const SimTK::Real xToes =  0.25;
        double radius = 0.04;
        double stiffness = 1e7;
        double dissipation = 1;
        double staticFriction = 0.6;
        double dynamicFriction = 0.4;
        double viscosity = 0.01;
        for (int icontact = 0; icontact < numContacts; ++icontact) {
            const SimTK::Real x = xHeel +
                    SimTK::Real(icontact) / SimTK::Real(numContacts - 1) *
                            (xToes - xHeel);
            const std::string geomName = "contact" + std::to_string(icontact);
            auto* contactGeom = new ContactSphere(radius, Vec3(x, 0, 0), calcn,
                    geomName);
            model.addContactGeometry(contactGeom);

            auto* params = new HuntCrossleyForce::ContactParameters(stiffness,
                    dissipation, staticFriction, dynamicFriction, viscosity);
            params->addGeometry(geomName);
            params->addGeometry("ground_halfspace");

            auto* contactForce = new HuntCrossleyForce(params);
            contactForce->setName(geomName + "_force");
            model.addForce(contactForce);
        }
    }


    // model.setUseVisualizer(true);
    // SimTK::State s = model.initSystem();
    // Manager manager(model, s);
    // manager.integrate(2.0);
    // std::cin.get();
    // std::exit(-1);

    // Kinematics data.
    // ----------------
    StatesTrajectory statesTraj;
    std::unique_ptr<Storage> motion;
    if (dataType == DataType::Treadmill) {
        const std::string trcFile = "sandboxCalibrateContact_markers.trc";
        const std::string motFile = "sandboxCalibrateContact.mot";
        auto ref = TRCFileAdapter::read("marker_trajectories.trc");
        // Convert from millimeters to meters.
        ref.updMatrix() /= 1000;
        const auto& reftime = ref.getIndependentColumn();
        const double walkingSpeed = 1.15; // m/s
        for (int i = 0; i < (int)ref.getNumColumns(); ++i) {
            SimTK::VectorView_<SimTK::Vec3> col =
                    ref.updDependentColumnAtIndex(i);
            for (int j = 0; j < col.size(); ++j) {
                col[j][0] += walkingSpeed * reftime[j]; // x
                col[j][1] -= 0.03; // y TODO
            }
        }
        TimeSeriesTable refFilt = filterLowpass(
                ref.flatten({"_x", "_y", "_z"}), 6.0, true);
        {
            // Convert back to millimeters.
            TimeSeriesTableVec3 refMM(ref);
            refMM.updMatrix() *= 1000;
            TRCFileAdapter::write(refMM, trcFile);
        }

        InverseKinematicsTool ikTool;
        Model modelForIK(model);
        ikTool.setModel(modelForIK);
        ikTool.setMarkerDataFileName(trcFile);
        ikTool.setOutputMotionFileName(motFile);
        ikTool.run();

        motion.reset(new Storage(motFile));

    } else if (dataType == DataType::Overground) {

        motion.reset(new Storage("loadedwalking_subject05_noload_free_"
                "trial03_ik_solution.mot"));

    }

    {
        // TODO motion.pad(motion.getSize() / 2);
        motion->lowpassIIR(6.0);
        // Estimate speeds from coordinates;
        // see AnalyzeTool::loadStatesFromFile().
        Storage* qStore = nullptr;
        Storage* uStore = nullptr;
        SimTK::State s = model.initSystem();
        model.getSimbodyEngine().formCompleteStorages(s, *motion, qStore,
                uStore);
        model.getSimbodyEngine().convertDegreesToRadians(*qStore);
        model.getSimbodyEngine().convertDegreesToRadians(*uStore);
        uStore->addToRdStorage(*qStore,
                qStore->getFirstTime(), qStore->getLastTime());
        motion.reset(new Storage(512, "states"));
        model.formStateStorage(*qStore, *motion.get(), false);
        delete qStore;
        delete uStore;
        int initialNumStates = motion->getSize();
        double duration = motion->getLastTime() - motion->getFirstTime();
        // Double the step size to reduce the number of states.
        motion->resample(2 * duration / (initialNumStates - 1), 5);
        statesTraj = StatesTrajectory::createFromStatesStorage(model,
                *motion, true);
        // visualize(model, *motion);
    }

    std::cout << "Number of states in trajectory: " << statesTraj.getSize()
            << std::endl;

    // IPOPT
    // -----
    /*
    ContactCalibration problem(model, statesTraj, numContacts);
    tropter::IPOPTSolver solver(problem);
    solver.set_verbosity(1);
    solver.set_max_iterations(100);
    //solver.set_hessian_approximation("exact");
    auto solution = solver.optimize();
    problem.applyParametersToModel(solution.variables, model);
    std::cout << solution.variables << std::endl;
    problem.printContactComparison(solution.variables,
            "sandboxCalibrateContact_comparison.sto");
    visualize(model, *motion);
     */

    // CMAES
    // -----
    SimTKContactCalibration sys(model, statesTraj, grfData, grfPrefix,
            contactModel, numContacts, optimizeIndividualMarkerHeights);
    SimTK::Vector results(sys.getNumParameters(), 0.5);
    SimTK::Optimizer opt(sys, SimTK::CMAES);
    opt.setMaxIterations(10000);
    opt.setDiagnosticsLevel(3);
    opt.setConvergenceTolerance(1e-3);
    opt.setAdvancedIntOption("popsize", 50);
    opt.setAdvancedRealOption("init_stepsize", 0.5);
    opt.setAdvancedStrOption("parallel", "multithreading");
    // To obtain repeatable results.
    opt.setAdvancedIntOption("seed", 42);
    opt.setAdvancedRealOption("maxTimeFractionForEigendecomposition", 1);
    Stopwatch watch;
    double f = opt.optimize(results);
    std::cout << "objective: " << f << std::endl;
    // std::cout << "variables: " << results << std::endl;
    std::cout << "Runtime: " << watch.getElapsedTimeFormatted() << std::endl;
    std::cout << "Number of states in trajectory: " << statesTraj.getSize()
            << std::endl;
    sys.printContactComparison(results,
            "sandboxCalibrateContact_comparison_cmaes.sto");
    sys.applyParametersToModel(results, model);
    visualize(model, *motion);
    std::string fileName = modelFileName;
    std::string ext = ".osim";
    fileName.replace(fileName.find(ext), ext.length(), "_opt" + ext);
    model.print(fileName);
}


void toyCMAES() {
    class OptSys : public SimTK::OptimizerSystem {
    public:
        OptSys() : SimTK::OptimizerSystem(2) {}
        int objectiveFunc(const SimTK::Vector& vars, bool, SimTK::Real& f)
                const override {
            ++m_count;
            const auto id = std::this_thread::get_id();
            if (m_map.count(id) == 0) m_map[id] = 0;
            m_map[id]++;
            std::stringstream ss;
            ss << "DEBUG " << m_count << " " << id;
            std::cout << ss.str() << std::endl;
            const double x = vars[0];
            const double y = vars[1];
            f = 0.5 * (3 * x * x + 4 * x * y + 6 * y * y) - 2 * x + 8 * y;
            return 0;
        }
        mutable std::atomic<int> m_count;
        mutable std::unordered_map<std::thread::id, int> m_map;
    };
    OptSys sys;
    SimTK::Vector results(2);
    SimTK::Optimizer opt(sys, SimTK::CMAES);
    opt.setAdvancedStrOption("parallel", "multithreading");
    double f = opt.optimize(results);
    std::cout << "objective: " << f << std::endl;
    std::cout << "variables: " << results << std::endl;
    for (const auto& entry : sys.m_map) {
        std::cout << entry.second << std::endl;
    }
}

int main() {

    // calibrateBall();

    // calibrateContact(DataType::Overground, ContactModel::AckermannVanDenBogert);
    calibrateContact(DataType::Treadmill, ContactModel::AckermannVanDenBogert,
            false);
    // calibrateContact(DataType::Treadmill, ContactModel::AckermannVanDenBogert);
    // calibrateContact(ContactModel::HuntCrossley);

    // OpenSim::LogBuffer::sync() is not threadsafe.
    // toyCMAES();

    return EXIT_SUCCESS;
}

