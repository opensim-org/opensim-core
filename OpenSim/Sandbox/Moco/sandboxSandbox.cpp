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

void createFeetCoordinatesReference(Model model, 
        const std::string& markersFile, const std::string& ikTasksFile) {
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
            TabOpConvertDegreesToRadians() |
            TabOpLowPassFilter(6) |
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
            const ContactBodyInfos& contactBodyInfos,
            const std::vector<std::string>& previousSolutionStates) {

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

        // Previous solution states.
        m_previousSolutionStateIndexes.resize(previousSolutionStates.size());
        for (int i = 0; i < (int)previousSolutionStates.size(); ++i) {
            auto it = std::find(m_stateNames.begin(), m_stateNames.end(), 
                    previousSolutionStates[i]);
            if (it == m_stateNames.end()) {
                throw Exception("State '{}' not found in state names.", 
                        previousSolutionStates[i]);
            }
            m_previousSolutionStateIndexes[i] = (int)(it - m_stateNames.begin());
        }

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

    void setPreviousSolution(const SimTK::RowVector& solution) {
        m_previousSolution = solution;
    }

    void setPreviousSolutionWeight(double weight) {
        m_previousSolutionWeight = weight;
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
                errors[ibody][ixyz] -= m_grfs.get(igrf++).calcValue(m_time);
            }
            forceError += m_forceScales[ibody] * errors[ibody].normSqr();
        }

        // Coordinate errors.
        double coordinateError = 0.0;
        double speedError = 0.0;
        int istate = 0;
        for (int iq = 0; iq < m_state.getNQ(); ++iq) {
            double error = q[iq] - m_states.get(istate).calcValue(m_time);
            coordinateError += m_stateWeightSet[istate] * error * error;
            istate++;
        }
        for (int iu = 0; iu < m_state.getNU(); ++iu) {
            double error = u[iu] - m_states.get(istate).calcValue(m_time);
            speedError += m_stateWeightSet[istate] * error * error;
            istate++;
        }

        // Previous solution error.
        double previousSolutionError = 0.0;
        for (int i = 0; i < (int)m_previousSolutionStateIndexes.size(); ++i) {
            double error;
            if (m_previousSolutionStateIndexes[i] < m_state.getNQ()) {
                error = q[m_previousSolutionStateIndexes[i]] - 
                        m_previousSolution[i]; 
            } else {
                error = u[m_previousSolutionStateIndexes[i] - m_state.getNQ()] - 
                        m_previousSolution[i];
            }
            previousSolutionError += error * error;
        }
           
        DM f = m_forceWeight * forceError + 
               m_stateWeight * (coordinateError / m_state.getNQ()) +
               m_stateWeight * (speedError / m_state.getNU()) +
               m_previousSolutionWeight * previousSolutionError;
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

    SimTK::RowVector m_previousSolution;
    std::vector<int> m_previousSolutionStateIndexes;
    double m_previousSolutionWeight = 1.0;
};

SimTK::RowVector solveContactTrackingProblem(ContactTrackingObjective& objfun, 
        double time, const TimeSeriesTable& states,
        SimTK::RowVector previousSolution) {

    // The number of states. For each foot, we have a 6 DOF joint between the 
    // ground and the calcaneus, and a 1 DOF joint between the calcaneus and
    // the toes.
    int nq = 14;      // number of generalized coordinates
    int nu = nq;      // number of generalized speeds
    int ns = nq + nu; // number of states

    // Define the optimization variables.
    MX q = MX::sym("q", nq, 1); // coordinates
    MX u = MX::sym("u", nu, 1); // speeds
    MX x = vertcat(q, u);       // states

    // Objective function
    objfun.setTime(time);
    objfun.setForceWeight(0.05);
    objfun.setTotalStateWeight(0.005);
    objfun.setStateWeight("/jointset/mtp_r/mtp_angle_r/value", 1e-6);
    objfun.setStateWeight("/jointset/mtp_l/mtp_angle_l/value", 1e-6);
    objfun.setStateWeight("/jointset/mtp_r/mtp_angle_r/speed", 1e-6);
    objfun.setStateWeight("/jointset/mtp_l/mtp_angle_l/speed", 1e-6);
    objfun.setPreviousSolution(previousSolution);
    objfun.setPreviousSolutionWeight(2.0);

    MXVector costOut;
    objfun.call({q, u}, costOut);
    MX f = MX::sum1(costOut.at(0));

    // Create the NLP.
    MXDict nlp;
    nlp.emplace(std::make_pair("x", x));
    nlp.emplace(std::make_pair("f", f));
    casadi::Dict opts;
    casadi::Dict optsIpopt;
    double tol = 1e-3;
    optsIpopt["tol"] = tol;
    optsIpopt["dual_inf_tol"] = tol;
    optsIpopt["compl_inf_tol"] = tol;
    optsIpopt["acceptable_tol"] = tol;
    optsIpopt["acceptable_dual_inf_tol"] = tol;
    optsIpopt["acceptable_compl_inf_tol"] = tol;
    opts["ipopt"] = optsIpopt;
    casadi::Function solver = nlpsol("solver", "ipopt", nlp, opts);

    // Set the bounds and initial guess.
    SimTK::RowVector row = states.getNearestRow(time);
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
    DMDict output = solver(input);

    // Print the optimal solution
    std::vector<double> xopt(output.at("x"));
    SimTK::RowVector solution(nq+nu, xopt.data());

    return solution;
}

void runContactTracking(Model contactModel, 
        const ContactBodyInfos& contactBodyInfos,
        const TimeSeriesTable& grfData, 
        const TimeSeriesTable& statesReference,
        double initialTime, double finalTime) {

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
    contactModel.initSystem();
    auto coordinateRefs = contactModel.getCoordinatesInMultibodyTreeOrder();
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
        std::cout << label << std::endl;
        states.appendColumn(label, 
                statesReference.getDependentColumn(label));
    }

    std::vector<std::string> previousSolutionStates = {
        "/jointset/mtp_r/mtp_angle_r/value",
        "/jointset/mtp_r/mtp_angle_r/speed",
        "/jointset/mtp_l/mtp_angle_l/value",
        "/jointset/mtp_l/mtp_angle_l/speed"};
    ContactTrackingObjective objfun(contactModel, grfs, states, contactBodyInfos,
            previousSolutionStates);

    const auto& times = states.getIndependentColumn();
    TimeSeriesTable statesSolutionTable;
    TimeSeriesTable forcesSolutionTable;

    SimTK::State state = contactModel.initSystem();
    int initialIndex = (int)states.getNearestRowIndexForTime(initialTime);
    int finalIndex = (int)states.getNearestRowIndexForTime(finalTime);
    SimTK::RowVector previousSolution((int)states.getNumColumns(), 0.0);
    for (int itime = initialIndex; itime <= finalIndex; ++itime) {
        double time = times[itime];
        SimTK::RowVector row = solveContactTrackingProblem(
                objfun, time, states, previousSolution);
        statesSolutionTable.appendRow(time, row);

        SimTK::Vector q(state.getNQ(), row.getContiguousScalarData(), true);
        SimTK::Vector u(state.getNU(), 
                row.getContiguousScalarData() + state.getNQ(), true);

        SimTK::Vector_<SimTK::Vec3> forces((int)contactBodyInfos.size(), 
                SimTK::Vec3(0.0));
        objfun.calcContactForces(contactModel, state, q, u, forces);

        SimTK::RowVector forcesRow((int)contactBodyInfos.size()*3);
        for (int i = 0; i < (int)contactBodyInfos.size(); ++i) {
            forcesRow[3*i] = forces[i][0];
            forcesRow[3*i+1] = forces[i][1];
            forcesRow[3*i+2] = forces[i][2];
        }

        forcesSolutionTable.appendRow(time, forcesRow);

        previousSolution = row;
    }
    statesSolutionTable.setColumnLabels(stateLabelsInOrder);
    statesSolutionTable.addTableMetaData<std::string>("inDegrees", "no");
    STOFileAdapter::write(statesSolutionTable, 
            "contact_initializer_solution_states.sto");

    forcesSolutionTable.setColumnLabels(grfLabelsInOrder);
    STOFileAdapter::write(forcesSolutionTable, 
            "contact_initializer_solution_forces.sto");
}

void computeFootMarkerPositions(Model contactModel, 
        const TimeSeriesTable& statesReference,
        const std::string& markersFile,
        const std::string& ikTasksFile) {
    contactModel.initSystem();
    auto statesTraj = StatesTrajectory::createFromStatesTable(contactModel, 
            statesReference);

    TimeSeriesTableVec3 markerTrajectories(markersFile);
    // TODO make sure a consistent time range is used everywhere
    markerTrajectories.trim(statesTraj.front().getTime(),
                             statesTraj.back().getTime());

    // TODO check if table has "Units"
    double scale = 1.0;
    const std::string& units = 
            markerTrajectories.getTableMetaData<std::string>("Units");
    if (units == "mm") {
        scale = 1000.0;
    } else if (units == "m") {
        scale = 1.0;
    } else {
        throw Exception("Units must be 'mm' or 'm'.");
    }

    auto markers = contactModel.getComponentList<Marker>();
    std::vector<std::string> markerNames;
    for (const auto& marker : markers) {
        markerNames.push_back(marker.getName());
    }

    TimeSeriesTableVec3 contactMarkerTrajectories;
    for (const auto& state : statesTraj) {
        contactModel.realizePosition(state);
        SimTK::RowVector_<SimTK::Vec3> row((int)markerNames.size());
        int imarker = 0;
        for (const auto& marker : markers) {
            SimTK::Vec3 location = marker.getLocationInGround(state);
            row[imarker++] = location * scale;
        }
        contactMarkerTrajectories.appendRow(state.getTime(), row);
    }
    contactMarkerTrajectories.setColumnLabels(markerNames);


    IKTaskSet ikTasks(ikTasksFile);
    std::vector<std::string> ikTaskNames;
    for (int i = 0; i < ikTasks.getSize(); ++i) {
        ikTaskNames.push_back(ikTasks.get(i).getName());
    }


    for (const auto& markerName : markerNames) {
        if (markerTrajectories.hasColumn(markerName)) {
            markerTrajectories.removeColumn(markerName);
        }
        markerTrajectories.appendColumn(markerName, 
                contactMarkerTrajectories.getDependentColumn(markerName));

        if (ikTaskNames.end() == std::find(ikTaskNames.begin(), 
                ikTaskNames.end(), markerName)) {
            IKMarkerTask ikTask;
            ikTask.setName(markerName);
            ikTask.setWeight(1000.0);  // TODO set relative to other weights
            ikTasks.cloneAndAppend(ikTask);
        } else {
            // TODO set relative to other weights
            ikTasks.get(markerName).setWeight(1000.0);
        }
    }

    TRCFileAdapter::write(markerTrajectories, 
            "marker_trajectories_updated.trc");
    ikTasks.print("ik_tasks_updated.xml");
}

void createUpdatedCoordinateReference(Model model, 
        const std::string& markersFile, const std::string& ikTasksFile) {
    // Run inverse kinematics on the full model.
    InverseKinematicsTool iktool;
    iktool.setModel(model);
    iktool.setMarkerDataFileName(markersFile);
    iktool.set_IKTaskSet(ikTasksFile);
    iktool.setOutputMotionFileName("full_model_ik.sto");    
    iktool.run();

    // Update the IK solution to something that we can visualize and use 
    // in Moco.
    TableProcessor tableProcessor = 
            TableProcessor("full_model_ik.sto") |
            TabOpUseAbsoluteStateNames() |
            TabOpConvertDegreesToRadians() |
            // TabOpLowPassFilter(10) |
            TabOpAppendCoordinateValueDerivativesAsSpeeds();

    TimeSeriesTable contactStatesReference = 
            tableProcessor.processAndConvertToRadians("", model);
    STOFileAdapter::write(contactStatesReference, 
            "coordinates_updated.sto");
}

int main() {

    // contactTracking();

    // Load original model.
    ModelProcessor modelProcessor = ModelProcessor("subject_walk_scaled.osim") |
        ModOpRemoveMuscles();
    Model model = modelProcessor.process();
    model.initSystem();

    // Load external loads.
    ExternalLoads externalLoads("grf_walk.xml", true);

    // Load IK data.
    std::string markersFile = "markers_walk.trc";
    std::string ikTasksFile = "ik_tasks_walk.xml";

    // Set time range;
    double initialTime = 0.48;
    // double initialTime = 0.75;
    double finalTime = 1.61;
    // double finalTime = 0.8;

    // Attach contact geometry and forces to the model.
    // TODO have user add this to the model?
    ContactGeometrySet contactGeometrySet("subject_walk_scaled_ContactGeometrySet.xml");
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

    // Detect the contact bodies.
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
    }

    // TODO check that the bodies with contact geometry are consistent with the 
    // external loads file.

    // input: original model (with contacts), contact body trees
    // output: "feet" model with contacts, markers etc.
    createContactModel(model, contactBodyInfos);

    // Print out contact body infos.
    for (const auto& info : contactBodyInfos) {
        std::cout << "Contact body: " << info.path << std::endl;
        std::cout << "Force identifier: " << info.force_indentifier << std::endl;
        for (const auto& contactForcePath : info.contact_force_paths) {
            std::cout << "Contact force: " << contactForcePath << std::endl;
        }
    }

    // Load the contacts model.
    Model contactModel(fmt::format("{}_contact_bodies.osim", model.getName()));
    contactModel.initSystem();

    // input: "feet" model, original TRC file, IK tasks
    // output: IK solution containing foot coordinates
    createFeetCoordinatesReference(contactModel, markersFile, ikTasksFile);

    // input: "feet" model, external loads file, coordinate reference file
    // output: states file, contact forces file (for debugging) 
    TimeSeriesTable contactStatesReference("contact_states_reference.sto");
    const std::string grfDataFilePath =
            convertRelativeFilePathToAbsoluteFromXMLDocument(
                    externalLoads.getDocumentFileName(),
                    externalLoads.getDataFileName());
    TimeSeriesTable grfData(grfDataFilePath);
    runContactTracking(contactModel, contactBodyInfos, grfData, 
            contactStatesReference, initialTime, finalTime);

    // input: "feet" model, states file
    // output: TRC file containing updated marker positions
    TimeSeriesTable contactStatesSolution(
            "contact_initializer_solution_states.sto");
    computeFootMarkerPositions(contactModel, contactStatesSolution,
            markersFile, ikTasksFile);

    // input: original model, "feet" TRC file, original TRC file, IK tasks
    createUpdatedCoordinateReference(model, "marker_trajectories_updated.trc",
            "ik_tasks_updated.xml");


    TimeSeriesTable coordinatesUpdated(
            "coordinates_updated.sto");
    VisualizerUtilities::showMotion(model, coordinatesUpdated);
    // TimeSeriesTableVec3 markerTrajectories("marker_trajectories_updated.trc");
    // VisualizerUtilities::showMarkerData(markerTrajectories);

    return EXIT_SUCCESS;
}

