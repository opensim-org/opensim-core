/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSandbox.cpp                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

// This file provides a way to easily prototype or test temporary snippets of
// code during development.

#include <OpenSim/Moco/osimMoco.h>
#include <casadi/casadi.hpp>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Common/TRCFileAdapter.h>
#include <OpenSim/Actuators/ModelProcessor.h>
#include <OpenSim/Actuators/ModelOperators.h>


using namespace OpenSim;
using namespace casadi;


using ContactForceRefs = std::vector<SimTK::ReferencePtr<const SmoothSphereHalfSpaceForce>>;

struct ContactBodyInfo {
    std::string path;
    std::string force_indentifier;
    std::vector<std::string> contact_force_paths;
};
using ContactBodyInfos = std::vector<ContactBodyInfo>;


void addContactsToModel(Model& model) {
    ContactGeometrySet contactGeometrySet(
            "subject_walk_scaled_ContactGeometrySet.xml");
    for (int i = 0; i < contactGeometrySet.getSize(); ++i) {
        ContactGeometry* contactGeometry = contactGeometrySet.get(i).clone();
        // Raise the ContactSpheres up by 2 cm so that the bottom of the spheres
        // are better aligned with the ground.
        if (contactGeometry->getName() != "floor") {
            SimTK::Vec3& location = contactGeometry->upd_location();
            location[1] += 0.02; 
        }
        model.addContactGeometry(contactGeometry);
    }
    model.finalizeConnections();

    ForceSet contactForceSet("subject_walk_scaled_ContactForceSet.xml");
    for (int i = 0; i < contactForceSet.getSize(); ++i) {
        model.addComponent(contactForceSet.get(i).clone());
    }
    model.finalizeConnections();
    model.initSystem();
}

ContactBodyInfos detectContactBodiesFromExternalLoads(const Model& model,
        const ExternalLoads& externalLoads) {
    ContactBodyInfos contactBodyInfos;
    for (int i = 0; i < externalLoads.getSize(); ++i) {
        ContactBodyInfo contactBodyInfo;
        const auto& externalForce = externalLoads.get(i);
        const auto& contactBodyName = externalForce.get_applied_to_body();
        std::string bodysetPath = fmt::format("/bodyset/{}", contactBodyName);
        std::string componentPath = fmt::format("/{}", contactBodyName);

        if (model.hasComponent<Body>(bodysetPath)) {
            contactBodyInfo.path = bodysetPath;
        } else if (model.hasComponent<Body>(componentPath)) {
            contactBodyInfo.path = componentPath;
        } else {
            OPENSIM_THROW(Exception, "Contact body '{}' not found in model.", 
                    contactBodyName);
        }

        contactBodyInfo.force_indentifier = externalForce.get_force_identifier();
        contactBodyInfo.contact_force_paths = {};
        contactBodyInfos.push_back(contactBodyInfo);

        log_info("Found contact body '{}'.", contactBodyName);
        log_info(" -- Path: '{}'.", contactBodyInfo.path);
        log_info(" -- Force identifier: '{}'.", contactBodyInfo.force_indentifier);
        log_info(" -- Contact bodies: ");
        const auto& paths = contactBodyInfo.contact_force_paths;
        for (int i = 0; i < (int)paths.size(); ++i) {
            log_info("    -- {}. {}", i+1, paths[i]);
        }
    }

    return contactBodyInfos;
}


void addGroundToFootJoint(Model& model, const std::string& footBodyName) {
    SpatialTransform transform;
    transform[0].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_rx", footBodyName), 1, 1));
    transform[0].setFunction(new LinearFunction());
    transform[0].setAxis(SimTK::Vec3(0, 0, 1));

    transform[1].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_ry", footBodyName), 1, 1));
    transform[1].setFunction(new LinearFunction());
    transform[1].setAxis(SimTK::Vec3(1, 0, 0));

    transform[2].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_rz", footBodyName), 1, 1));
    transform[2].setFunction(new LinearFunction());
    transform[2].setAxis(SimTK::Vec3(0, 1, 0));

    transform[3].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_tx", footBodyName), 1, 1));
    transform[3].setFunction(new LinearFunction());
    transform[3].setAxis(SimTK::Vec3(1, 0, 0));

    transform[4].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_ty", footBodyName), 1, 1));
    transform[4].setFunction(new LinearFunction());
    transform[4].setAxis(SimTK::Vec3(0, 1, 0));

    transform[5].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_tz", footBodyName), 1, 1));
    transform[5].setFunction(new LinearFunction());
    transform[5].setAxis(SimTK::Vec3(0, 0, 1));

    CustomJoint* joint = new CustomJoint(
            fmt::format("ground_{}", footBodyName), 
            model.getGround(), 
            model.getComponent<Body>(fmt::format("/bodyset/{}", footBodyName)), 
            transform);
    model.addJoint(joint);
}

void copyMarkersForBody(const Model& model, Model& contactModel, 
        const std::string& bodyPath) {
    for (const auto& marker : model.getComponentList<Marker>()) {
        if (marker.getSocket<PhysicalFrame>("parent_frame").getConnecteePath()
                == bodyPath) {
            contactModel.addMarker(marker.clone());
        }
    }
}

void copySmoothSphereHalfSpaceForceForContactGeometry(const Model& model, 
        Model& contactModel, const ContactGeometry& geometry,
        ContactBodyInfo& contactBodyInfo) {
    for (const auto& force : model.getComponentList<SmoothSphereHalfSpaceForce>()) {
        if (force.getSocket<ContactGeometry>("sphere").getConnecteePath()
                == geometry.getAbsolutePathString()) {
            
            // Add in the contact force and the half space geometry.
            const ContactGeometry& halfSpace = 
                    force.getSocket<ContactGeometry>("half_space").getConnectee();
            const std::string& halfSpacePath = halfSpace.getAbsolutePathString();
            if (!contactModel.hasComponent<ContactGeometry>(halfSpacePath)) {
                contactModel.addContactGeometry(halfSpace.clone());
            }

            const std::string& forcePath = force.getAbsolutePathString();
            if (forcePath.find("/forceset/") != std::string::npos) {
                contactModel.addForce(force.clone());
            } else {
                contactModel.addComponent(force.clone());
            }

            contactBodyInfo.contact_force_paths.push_back(
                    force.getAbsolutePathString());
        }
    }
}

void copyContactGeometryForBody(const Model& model, const std::string& bodyPath,
         Model& contactModel, ContactBodyInfo& contactBodyInfo) {
    for (const auto& geometry : model.getComponentList<ContactGeometry>()) {
        if (geometry.getSocket<PhysicalFrame>("frame").getConnecteePath()
                == bodyPath) {
            contactModel.addContactGeometry(geometry.clone());
            copySmoothSphereHalfSpaceForceForContactGeometry(model, 
                    contactModel, geometry, contactBodyInfo);
        }
    }
}

void copyDependentBodiesAndJoints(const Model& model, 
        const std::string& bodyPath, Model& contactModel,
        ContactBodyInfo& contactBodyInfo) {
    for (const auto& joint : model.getComponentList<Joint>()) {
        const Frame& parent = joint.getParentFrame().findBaseFrame();
        const std::string& parentPath = parent.getAbsolutePathString();

        // If the parent of the joint matches the body, copy the joint and the
        // child body of the joint.
        if (parentPath == bodyPath) {
            const Frame& child = joint.getChildFrame().findBaseFrame();
            const Body& body = model.getComponent<Body>(
                    child.getAbsolutePathString());

            // The body must be added before the joint.
            contactModel.addBody(body.clone());
            contactModel.addJoint(joint.clone());

            copyMarkersForBody(model, contactModel, 
                    body.getAbsolutePathString());
            copyContactGeometryForBody(model, body.getAbsolutePathString(), 
                    contactModel, contactBodyInfo);

            // Recursively copy the children of the child body.
            copyDependentBodiesAndJoints(model, body.getAbsolutePathString(), 
                   contactModel, contactBodyInfo);
        }
    }
}

void createContactModel(const Model& model, ContactBodyInfos& contactBodyInfos) {
    
    Model contactModel;
    for (auto& info : contactBodyInfos) {
        Body* body = model.getComponent<Body>(info.path).clone();
        contactModel.addBody(body);
        copyMarkersForBody(model, contactModel, info.path);
        copyContactGeometryForBody(model, info.path, contactModel, info);
        copyDependentBodiesAndJoints(model, info.path, contactModel, info);
        addGroundToFootJoint(contactModel, body->getName());
    }

    contactModel.finalizeConnections();
    contactModel.print(fmt::format("{}_contact_bodies.osim", model.getName()));
}

void createContactModelReference(Model model, 
        const std::string& markersFile, const std::string& ikTasksFile,
        double lowPassFilterFrequency) {
    // Run inverse kinematics on the foot model.
    InverseKinematicsTool iktool;
    iktool.setModel(model);
    iktool.setMarkerDataFileName(markersFile);
    iktool.set_IKTaskSet(IKTaskSet(ikTasksFile));
    iktool.setOutputMotionFileName("contact_bodies_ik.sto");    
    iktool.run();

    // Update the IK solution to something that we can visualize and use 
    // in Moco.
    TableProcessor tableProcessor = 
            TableProcessor("contact_bodies_ik.sto") |
            TabOpUseAbsoluteStateNames() |
            TabOpLowPassFilter(lowPassFilterFrequency) |
            TabOpConvertDegreesToRadians() |
            TabOpAppendCoordinateValueDerivativesAsSpeeds();

    TimeSeriesTable contactStatesReference = 
            tableProcessor.processAndConvertToRadians("", model);
    STOFileAdapter::write(contactStatesReference, 
            "contact_states_reference.sto");
}

class ContactTrackingObjective : public Callback {
public:
    ContactTrackingObjective(Model model,
            TimeSeriesTable grfs, TimeSeriesTable states,
            const ContactBodyInfos& contactBodyInfos) {

        // Model.
        m_model = std::move(model);
        m_state = m_model.initSystem();
        
        // Contact forces.
        for (int ibody = 0; ibody < (int)contactBodyInfos.size(); ++ibody) {
            const auto& info = contactBodyInfos[ibody];
            for (const auto& contactForcePath : info.contact_force_paths) {
                m_contactForceRefs.emplace_back( 
                        &m_model.getComponent<SmoothSphereHalfSpaceForce>(
                                contactForcePath));
                m_bodyIndexes.push_back(ibody);
            }
            numContactBodies++;
        }

        // Ground reaction forces.
        m_grfs = GCVSplineSet(grfs);
        std::vector<std::string> suffixes = {"x", "y", "z"};
        TimeSeriesTableVec3 grfsPacked = grfs.pack<SimTK::Vec3>(suffixes);
        std::vector<double> peakMagnitudes(grfsPacked.getNumColumns(), 0.0);
        for (int itime = 0; itime < (int)grfsPacked.getNumRows(); ++itime) {
            const auto row = grfsPacked.getRowAtIndex(itime);
            for (int igrf = 0; igrf < row.size(); ++igrf) {
                peakMagnitudes[igrf] = std::max(peakMagnitudes[igrf], 
                        row[igrf].normSqr());
            }
        }
        for (const auto& mag : peakMagnitudes) {
            m_forceScales.push_back(1.0 / mag);
        }

        // States.
        m_stateNames = states.getColumnLabels();
        for (const auto& stateName : m_stateNames) {
            m_stateWeightSet.push_back(1.0);
        }
        m_states = GCVSplineSet(states);

        casadi::Dict opts;
        opts["enable_fd"] = true;
        this->construct("objective", opts);
    }

    void setTime(double time) {
        m_time = SimTK::Vector(1, time);
    }

    void setForceWeight(double weight) {
        m_forceWeight = weight;
    }

    void setTotalStateWeight(double weight) {
        m_stateWeight = weight;
    }

    void setStateWeight(const std::string& stateName, double weight) {
        auto it = std::find(m_stateNames.begin(), m_stateNames.end(), stateName);
        if (it == m_stateNames.end()) {
            throw Exception("State '{}' not found in state names.", stateName);
        }
        m_stateWeightSet[it - m_stateNames.begin()] = weight;
    }

    void setPreviousStates(const SimTK::RowVector& prevStates) {
        m_previousStates = prevStates;
    }

    void setPreviousStatesWeight(double weight) {
        m_previousStatesWeight = weight;
    }

    const ContactForceRefs& getContactForceRefs() const {
        return m_contactForceRefs;
    }

    void calcContactForces(const Model& model, SimTK::State& state, 
            const SimTK::Vector& q, const SimTK::Vector& u,
            SimTK::Vector_<SimTK::Vec3>& forces) const {
        state.updQ() = q;
        state.updU() = u;
        model.realizeVelocity(state);
        int iforce = 0;
        for (const auto& contactForce : m_contactForceRefs) {
            forces[m_bodyIndexes[iforce++]] += 
                    contactForce->getSphereForce(state)[1];
        }
    }

    // Number of inputs and outputs
    casadi_int get_n_in() override { return 2;}
    casadi_int get_n_out() override { return 1;}

    Sparsity get_sparsity_in(casadi_int i) override {
        if (i == 0) {
            return Sparsity::dense(m_state.getNQ(), 1);
        } else if (i == 1) {
            return Sparsity::dense(m_state.getNU(), 1);
        } else {
            return Sparsity::dense(0, 0);
        }
    }

    Sparsity get_sparsity_out(casadi_int i) override {
        if (i == 0) {
            return Sparsity::dense(1, 1);
        } else {
            return Sparsity::dense(0, 0);
        }
    }

    // Evaluate numerically
    std::vector<DM> eval(const std::vector<DM>& arg) const override {        
        SimTK::Vector q(m_state.getNQ(), arg.at(0)->data(), true);
        SimTK::Vector u(m_state.getNU(), arg.at(1)->data(), true);

        SimTK::Vector_<SimTK::Vec3> errors(numContactBodies, SimTK::Vec3(0.0));
        calcContactForces(m_model, m_state, q, u, errors);

        // Force errors.
        int igrf = 0;
        double forceError = 0;
        for (int ibody = 0; ibody < numContactBodies; ++ibody) {
            for (int ixyz = 0; ixyz < 3; ++ixyz) {
                errors[ibody][ixyz] -= m_grfs.get(igrf).calcValue(m_time);
                igrf++;
            }
            forceError += m_forceScales[ibody] * errors[ibody].normSqr();
        }

        // Coordinate errors.
        double previousStatesError = 0.0;
        double coordinateError = 0.0;
        double speedError = 0.0;
        int istate = 0;
        for (int iq = 0; iq < m_state.getNQ(); ++iq) {
            double error = q[iq] - m_states.get(istate).calcValue(m_time);
            coordinateError += m_stateWeightSet[istate] * error * error;
            double prevError = q[iq] - m_previousStates[istate];
            previousStatesError += prevError * prevError;
            istate++;
        }
        for (int iu = 0; iu < m_state.getNU(); ++iu) {
            double error = u[iu] - m_states.get(istate).calcValue(m_time);
            speedError += m_stateWeightSet[istate] * error * error;
            double prevError = u[iu] - m_previousStates[istate];
            previousStatesError += prevError * prevError;
            istate++;
        }
           
        DM f = m_forceWeight * forceError + 
               m_stateWeight * (coordinateError / m_state.getNQ()) +
               m_stateWeight * (speedError / m_state.getNU()) +
               m_previousStatesWeight * previousStatesError;
        return {f};
    }

private:
    Model m_model;
    mutable SimTK::State m_state;
    SimTK::Vector m_time;

    GCVSplineSet m_grfs;
    GCVSplineSet m_states;

    int numContactBodies = 0;
    ContactForceRefs m_contactForceRefs;
    std::vector<int> m_bodyIndexes;

    std::vector<double> m_forceScales;
    double m_forceWeight = 1.0;
    double m_stateWeight = 1.0;
    std::vector<std::string> m_stateNames;
    std::vector<double> m_stateWeightSet;

    SimTK::RowVector m_previousStates;
    double m_previousStatesWeight = 1.0;
};

SimTK::RowVector solveContactTrackingProblem(int nq, int nu, 
        double time, const TimeSeriesTable& states,
        const std::unordered_map<std::string, double>& weights,
        SimTK::RowVector previousStates, 
        ContactTrackingObjective& objfun) {

    // The number of states. For each foot, we have a 6 DOF joint between the 
    // ground and the calcaneus, and a 1 DOF joint between the calcaneus and
    // the toes.
    int ns = nq + nu; // number of states

    // Define the optimization variables.
    MX q = MX::sym("q", nq, 1); // coordinates
    MX u = MX::sym("u", nu, 1); // speeds
    MX x = vertcat(q, u);       // states

    // Objective function
    objfun.setTime(time);
    objfun.setForceWeight(weights.at("force"));
    objfun.setTotalStateWeight(weights.at("state"));
    objfun.setPreviousStates(previousStates);
    objfun.setPreviousStatesWeight(weights.at("previous_states"));

    MXVector costOut;
    objfun.call({q, u}, costOut);
    MX f = MX::sum1(costOut.at(0));

    // Create the NLP.
    MXDict nlp;
    nlp.emplace(std::make_pair("x", x));
    nlp.emplace(std::make_pair("f", f));
    casadi::Dict opts;
    casadi::Dict optsIpopt;
    double tol = 1e-4;
    optsIpopt["tol"] = tol;
    optsIpopt["dual_inf_tol"] = tol;
    optsIpopt["compl_inf_tol"] = tol;
    optsIpopt["acceptable_tol"] = tol;
    optsIpopt["acceptable_dual_inf_tol"] = tol;
    optsIpopt["acceptable_compl_inf_tol"] = tol;
    opts["ipopt"] = optsIpopt;
    casadi::Function solver = nlpsol("solver", "ipopt", nlp, opts);

    // Set the bounds and initial guess.
    double objective = SimTK::Infinity;
    bool perturbRow = false;
    DMDict output;
    while (objective > 1e2) {
        SimTK::RowVector row = previousStates;
        if (perturbRow) {
            row += SimTK::Test::randVector(row.size()).transpose();
        }
        std::vector<double> x0;
        std::vector<double> lbx;
        std::vector<double> ubx;
        for (int i = 0; i < row.size(); ++i) {
            x0.push_back(row[i]);
            lbx.push_back(-std::numeric_limits<double>::infinity());
            ubx.push_back(std::numeric_limits<double>::infinity());
        }

        // Solve the problem
        DMDict input = {{"lbx", lbx},
                        {"ubx", ubx},
                        {"x0",  x0}};
        output = solver(input);
        perturbRow = true;
        objective = output.at("f").scalar();
    }

    // Print the optimal solution
    std::vector<double> xopt(output.at("x"));
    SimTK::RowVector solution(nq+nu, xopt.data());

    return solution;
}

void runContactTracking(Model model, 
        const ContactBodyInfos& contactBodyInfos,
        const TimeSeriesTable& grfData, 
        const TimeSeriesTable& statesReference,
        const std::unordered_map<std::string, double>& weights, 
        const std::pair<double, double>& timeRange) {

    std::vector<std::string> grfLabelsInOrder;
    for (const auto& info : contactBodyInfos) {
        grfLabelsInOrder.push_back(info.force_indentifier + "x");
        grfLabelsInOrder.push_back(info.force_indentifier + "y");
        grfLabelsInOrder.push_back(info.force_indentifier + "z");
    }

    TimeSeriesTable grfs(grfData.getIndependentColumn());
    for (const auto& label : grfLabelsInOrder) {
        grfs.appendColumn(label, grfData.getDependentColumn(label));
    }

    std::vector<std::string> stateLabelsInOrder;
    model.initSystem();
    auto coordinateRefs = model.getCoordinatesInMultibodyTreeOrder();
    for (int i = 0; i < (int)coordinateRefs.size(); ++i) {
        std::string valuePath = fmt::format("{}/value", 
                coordinateRefs[i]->getAbsolutePathString());
        stateLabelsInOrder.push_back(valuePath);
    }
    for (int i = 0; i < (int)coordinateRefs.size(); ++i) {
        std::string speedPath = fmt::format("{}/speed", 
                coordinateRefs[i]->getAbsolutePathString());
        stateLabelsInOrder.push_back(speedPath);
    }

    TimeSeriesTable states(statesReference.getIndependentColumn());
    for (const auto& label : stateLabelsInOrder) {
        states.appendColumn(label, 
                statesReference.getDependentColumn(label));
    }

    ContactTrackingObjective objfun(model, grfs, states, contactBodyInfos);

    const auto& times = states.getIndependentColumn();
    TimeSeriesTable statesSolutionTable;
    TimeSeriesTable forcesSolutionTable;

    SimTK::State state = model.initSystem();
    int nq = state.getNQ();
    int nu = state.getNU();
    int initialIndex = (int)states.getNearestRowIndexForTime(timeRange.first);
    int finalIndex = (int)states.getNearestRowIndexForTime(timeRange.second);
    SimTK::RowVector previousStates = states.getRowAtIndex(initialIndex);
    for (int itime = initialIndex; itime <= finalIndex; ++itime) {
        double time = times[itime];
        SimTK::RowVector statesRow = solveContactTrackingProblem(
                nq, nu, time, states, weights, previousStates, objfun);
        statesSolutionTable.appendRow(time, statesRow);

        SimTK::Vector q(state.getNQ(), statesRow.getContiguousScalarData(), true);
        SimTK::Vector u(state.getNU(), 
                statesRow.getContiguousScalarData() + state.getNQ(), true);

        SimTK::Vector_<SimTK::Vec3> forces((int)contactBodyInfos.size(), 
                SimTK::Vec3(0.0));
        objfun.calcContactForces(model, state, q, u, forces);

        SimTK::RowVector forcesRow((int)contactBodyInfos.size()*3);
        for (int i = 0; i < (int)contactBodyInfos.size(); ++i) {
            forcesRow[3*i] = forces[i][0];
            forcesRow[3*i+1] = forces[i][1];
            forcesRow[3*i+2] = forces[i][2];
        }

        forcesSolutionTable.appendRow(time, forcesRow);

        previousStates = statesRow;
    }
    statesSolutionTable.setColumnLabels(stateLabelsInOrder);
    statesSolutionTable.addTableMetaData<std::string>("inDegrees", "no");
    STOFileAdapter::write(statesSolutionTable, 
            fmt::format("contact_initializer_solution_states.sto"));

    forcesSolutionTable.setColumnLabels(grfLabelsInOrder);
    STOFileAdapter::write(forcesSolutionTable, 
            fmt::format("contact_initializer_solution_forces.sto"));
}

void calcOrientationsAndTranslationsFromContactSolution(Model contactModel,
        double lowPassFilterFrequency) {
    contactModel.initSystem();

    TableProcessor tableProcessor = 
            TableProcessor("contact_initializer_solution_states.sto") |
            TabOpLowPassFilter(lowPassFilterFrequency);
    TimeSeriesTable contactStatesSolutionFiltered =
            tableProcessor.process(&contactModel);

    STOFileAdapter::write(contactStatesSolutionFiltered, 
            "contact_initializer_solution_states_filtered.sto");

    auto statesTraj = StatesTrajectory::createFromStatesTable(
            contactModel, contactStatesSolutionFiltered, true, true, false);

    std::vector<std::string> frame_paths;
    frame_paths.push_back("/bodyset/calcn_r");
    frame_paths.push_back("/bodyset/toes_r");
    frame_paths.push_back("/bodyset/calcn_l");
    frame_paths.push_back("/bodyset/toes_l");

    TimeSeriesTable_<SimTK::Quaternion> orientations;
    for (auto state : statesTraj) {
        contactModel.realizePosition(state);
        std::vector<SimTK::Quaternion> quaternions;
        for (const auto& path : frame_paths) {
            SimTK::Quaternion quaternion =
                contactModel.getComponent<Frame>(path).getRotationInGround(state)
                    .convertRotationToQuaternion();
            quaternions.push_back(quaternion);
        }
        orientations.appendRow(state.getTime(), quaternions);
    }
    orientations.setColumnLabels(frame_paths);

    STOFileAdapter_<SimTK::Quaternion>::write(orientations, 
            "contact_initializer_solution_orientations.sto");

    TimeSeriesTable_<SimTK::Vec3> angular_velocities;
    for (auto state : statesTraj) {
        contactModel.realizeVelocity(state);
        std::vector<SimTK::Vec3> angular_velocity_vector;
        for (const auto& path : frame_paths) {
            SimTK::Vec3 angular_velocity =
                contactModel.getComponent<Frame>(path).getAngularVelocityInGround(state);
            angular_velocity_vector.push_back(angular_velocity);
        }
        angular_velocities.appendRow(state.getTime(), angular_velocity_vector);
    }
    angular_velocities.setColumnLabels(frame_paths);

    STOFileAdapter_<SimTK::Vec3>::write(angular_velocities, 
            "contact_initializer_solution_angular_velocities.sto");


    TimeSeriesTable_<SimTK::Vec3> translations;
    for (auto state : statesTraj) {
        contactModel.realizePosition(state);
        std::vector<SimTK::Vec3> locations;
        for (const auto& path : frame_paths) {
            SimTK::Vec3 location =
                contactModel.getComponent<Frame>(path).getPositionInGround(state);
            locations.push_back(location);
        }
        translations.appendRow(state.getTime(), locations);
    }
    translations.setColumnLabels(frame_paths);

    STOFileAdapter_<SimTK::Vec3>::write(translations, 
            "contact_initializer_solution_translations.sto");
}

int main() {

    // Load the original model.
    ModelProcessor modelProcessor = ModelProcessor("subject_walk_scaled.osim") |
        ModOpRemoveMuscles();
    Model model = modelProcessor.process();
    model.initSystem();

    // Load external loads.
    ExternalLoads externalLoads("grf_walk.xml", true);

    // Load IK data.
    std::string markersFile = "markers_walk.trc";
    std::string ikTasksFile = "ik_tasks_walk.xml";

    // Set the time range.
    std::pair<double, double> timeRange = std::make_pair(0.48, 1.61);

    // Set cost function weights.    
    std::unordered_map<std::string, double> weights;
    weights["force"] = 0.1;
    weights["state"] = 0.01;
    weights["previous_states"] = 1e-3;

    // Low-pass filter frequency.
    // double lowPassFilterFrequency = 10.0;

    // Attach contact geometry and forces to the model.
    addContactsToModel(model);

    // Detect the contact bodies.
    ContactBodyInfos contactBodyInfos = 
            detectContactBodiesFromExternalLoads(model, externalLoads);

    // Create a minimal model containing only the "contact" bodies based on the
    // external loads file and bodies down the kinematic chain. Also copy the
    // contact forces, contact geometry, and markers associated with these 
    // bodies to the new model. A 6 DOF joint is added between the ground and
    // each contact body.
    createContactModel(model, contactBodyInfos);

    // Load the contacts model.
    Model contactModel(fmt::format("{}_contact_bodies.osim", model.getName()));
    contactModel.initSystem();

    // Run inverse kinematics with the contact model to get a reference set of 
    // coordinate values and speeds to track the external loads.
    createContactModelReference(contactModel, markersFile, ikTasksFile, 
            6.0);

    // Run contact tracking to find the states that best match the external
    // loads.
    TimeSeriesTable contactStatesReference("contact_states_reference.sto"); 
    const std::string grfDataFilePath =
            convertRelativeFilePathToAbsoluteFromXMLDocument(
                    externalLoads.getDocumentFileName(),
                    externalLoads.getDataFileName());
    TimeSeriesTable grfData(grfDataFilePath);
    runContactTracking(contactModel, contactBodyInfos, grfData, 
            contactStatesReference, weights, timeRange);

    // Calculate the body orientation and translations from the contact tracking
    // solution
    calcOrientationsAndTranslationsFromContactSolution(contactModel, 
            10.0);

    // TODOs
    // -- check that the bodies with contact geometry are consistent with the external loads file.

    return EXIT_SUCCESS;
}


// Unused code
// ----------------

// // Add additional markers to the models to encode more orientation 
// // information from the contact tracking solution.
// addAdditionalMarkers(contactModel, model);

// // input: "feet" model, states file
// // output: TRC file containing updated marker positions
// TimeSeriesTable contactStatesSolution(
//         "contact_initializer_solution_states_contact_model.sto");
// computeContactBodyMarkerPositions(contactModel, contactStatesSolution,
//         markersFile, ikTasksFile);

// // input: original model, "feet" TRC file, original TRC file, IK tasks
// createUpdatedCoordinateReference(model, "markers_walk_updated.trc",
//         "ik_tasks_walk_updated.xml", -1.0);


// // Calculate the contact forces from the final solution.
// TimeSeriesTable coordinatesUpdated("coordinates_updated.sto");
// TimeSeriesTable empty(coordinatesUpdated.getIndependentColumn());
// std::vector<std::string> outputPaths;
// outputPaths.push_back(".*sphere_force.*");
// TimeSeriesTable_<SimTK::SpatialVec> forces =
//         analyze<SimTK::SpatialVec>(model, coordinatesUpdated, empty, outputPaths);

// TimeSeriesTable forcesFlat = forces.flatten();
// STOFileAdapter::write(forcesFlat, "forces_flat.sto");


// std::vector<std::string> contactForcesRight = {"/contactHeel_r", 
//         "/contactLateralRearfoot_r", "/contactLateralMidfoot_r", 
//         "/contactMedialMidfoot_r", "/contactLateralToe_r", 
//         "/contactMedialToe_r"};
// contactBodyInfos[0].contact_force_paths = contactForcesRight;

// std::vector<std::string> contactForcesLeft = {"/contactHeel_l", 
//         "/contactLateralRearfoot_l", "/contactLateralMidfoot_l", 
//         "/contactMedialMidfoot_l", "/contactLateralToe_l", 
//         "/contactMedialToe_l"};
// contactBodyInfos[1].contact_force_paths = contactForcesLeft;


// std::vector<std::string> suffixes = {"x", "y", "z"};
// TimeSeriesTable forcesFromFinalSolution(
//         forcesFlat.getIndependentColumn());
// for (const auto& info : contactBodyInfos) {
//     const auto& identifier = info.force_indentifier;
//     for (int i = 0; i < 3; ++i) {
//         std::string label = fmt::format("{}{}", identifier, suffixes[i]);
//         SimTK::Vector column((int)forcesFlat.getNumRows(), 0.0);
//         for (const auto& path : info.contact_force_paths) {
//             column += forcesFlat.getDependentColumn(
//                 fmt::format("{}|sphere_force_{}", path, i+4));
//         }
//         forcesFromFinalSolution.appendColumn(label, column);      
//     }
// }

// STOFileAdapter::write(forcesFromFinalSolution, 
//         "forces_from_updated_coordinates.sto");

// VisualizerUtilities::showMotion(model, coordinatesUpdated);

// TimeSeriesTableVec3 markerTrajectories("markers_walk_updated.trc");
// VisualizerUtilities::showMarkerData(markerTrajectories);

// void computeContactBodyMarkerPositions(Model contactModel, 
//         const TimeSeriesTable& statesReference,
//         const std::string& markersFile,
//         const std::string& ikTasksFile) {
//     contactModel.initSystem();
//     auto statesTraj = StatesTrajectory::createFromStatesTable(contactModel, 
//             statesReference);

//     TimeSeriesTableVec3 markerTrajectories(markersFile);
//     // TODO make sure a consistent time range is used everywhere
//     markerTrajectories.trim(statesTraj.front().getTime(),
//                              statesTraj.back().getTime());

//     // TODO check if table has "Units"
//     double scale = 1.0;
//     const std::string& units = 
//             markerTrajectories.getTableMetaData<std::string>("Units");
//     if (units == "mm") {
//         scale = 1000.0;
//     } else if (units == "m") {
//         scale = 1.0;
//     } else {
//         throw Exception("Units must be 'mm' or 'm'.");
//     }

//     auto markers = contactModel.getComponentList<Marker>();
//     std::vector<std::string> markerNames;
//     for (const auto& marker : markers) {
//         markerNames.push_back(marker.getName());
//     }

//     TimeSeriesTableVec3 contactMarkerTrajectories;
//     for (const auto& state : statesTraj) {
//         contactModel.realizePosition(state);
//         SimTK::RowVector_<SimTK::Vec3> row((int)markerNames.size());
//         int imarker = 0;
//         for (const auto& marker : markers) {
//             SimTK::Vec3 location = marker.getLocationInGround(state);
//             row[imarker++] = location * scale;
//         }
//         contactMarkerTrajectories.appendRow(state.getTime(), row);
//     }
//     contactMarkerTrajectories.setColumnLabels(markerNames);


//     IKTaskSet ikTasks(ikTasksFile);
//     std::vector<std::string> ikTaskNames;
//     for (int i = 0; i < ikTasks.getSize(); ++i) {
//         ikTaskNames.push_back(ikTasks.get(i).getName());
//     }


//     for (const auto& markerName : markerNames) {
//         if (markerTrajectories.hasColumn(markerName)) {
//             markerTrajectories.removeColumn(markerName);
//         }
//         markerTrajectories.appendColumn(markerName, 
//                 contactMarkerTrajectories.getDependentColumn(markerName));

//         if (ikTaskNames.end() == std::find(ikTaskNames.begin(), 
//                 ikTaskNames.end(), markerName)) {
//             IKMarkerTask ikTask;
//             ikTask.setName(markerName);
//             ikTask.setWeight(10.0);  // TODO set relative to other weights
//             ikTasks.cloneAndAppend(ikTask);
//         } else {
//             // TODO set relative to other weights
//             ikTasks.get(markerName).setWeight(10.0);
//         }
//     }

//     TRCFileAdapter::write(markerTrajectories, "markers_walk_updated.trc");
//     ikTasks.print("ik_tasks_walk_updated.xml");
// }

// void createUpdatedCoordinateReference(Model model, 
//         const std::string& markersFile, const std::string& ikTasksFile,
//         double lowPassFilterFrequency) {
//     // Run inverse kinematics on the full model.
//     InverseKinematicsTool iktool;
//     iktool.setModel(model);
//     iktool.setMarkerDataFileName(markersFile);
//     iktool.set_IKTaskSet(ikTasksFile);
//     iktool.setOutputMotionFileName("full_model_ik.sto");    
//     iktool.run();

//     // Update the IK solution to something that we can visualize and use 
//     // in Moco.
//     TableProcessor tableProcessor = 
//             TableProcessor("full_model_ik.sto") |
//             TabOpUseAbsoluteStateNames() |
//             TabOpConvertDegreesToRadians() |
//             TabOpLowPassFilter(lowPassFilterFrequency) |
//             TabOpAppendCoordinateValueDerivativesAsSpeeds();

//     TimeSeriesTable contactStatesReference = 
//             tableProcessor.processAndConvertToRadians("", model);
//     STOFileAdapter::write(contactStatesReference, 
//             "coordinates_updated.sto");
// }

// void addAdditionalMarkers(Model& contactModel, Model& model) {

//     double offset = 0.06;
//     std::vector<std::string> offsets = {"x", "y", "z"};

//     std::vector<std::string> markerPaths;
//     for (const auto& marker : contactModel.getComponentList<Marker>()) {
//         markerPaths.push_back(marker.getAbsolutePathString());
//     }

//     for (const auto& markerPath : markerPaths) {
//         for (int i = 0; i < 3; ++i) {
//             const auto& marker = contactModel.getComponent<Marker>(markerPath);
//             std::string offsetName = fmt::format("{}_{}", marker.getName(), offsets[i]);
//             SimTK::Vec3 location = marker.get_location();
//             location[i] += offset;
//             Marker* offsetMarker = new Marker(offsetName, 
//                     marker.getParentFrame(), 
//                     location);
//             contactModel.addMarker(offsetMarker);
//         }
//     }
//     contactModel.finalizeConnections();

//     for (const auto& markerPath : markerPaths) {
//         for (int i = 0; i < 3; ++i) {
//             const auto& marker = model.getComponent<Marker>(markerPath);
//             std::string offsetName = fmt::format("{}_{}", marker.getName(), offsets[i]);
//             SimTK::Vec3 location = marker.get_location();
//             location[i] += offset;
//             Marker* offsetMarker = new Marker(offsetName, 
//                     marker.getParentFrame(), 
//                     location);
//             model.addMarker(offsetMarker);
//         }
//     }
//     model.finalizeConnections();
// }

