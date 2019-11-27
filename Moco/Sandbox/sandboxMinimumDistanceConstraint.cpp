/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxMinimumDistanceConstraint.cpp                         *
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

#include <Moco/osimMoco.h>

using namespace OpenSim;
using SimTK::Vec3;

Model createModel() {
    Model model;

    auto* body = new Body("body", 1, Vec3(0), SimTK::Inertia(1));
    model.addBody(body);

    auto* joint = new GimbalJoint("gimbal", model.getGround(), 
            Vec3(0, 1, 0), Vec3(0, 0, -SimTK::Pi / 2), *body, Vec3(-1, 0, 0), 
            Vec3(0));
    auto& qx = joint->updCoordinate(GimbalJoint::Coord::Rotation1X);
    qx.setName("qx");
    auto& qy = joint->updCoordinate(GimbalJoint::Coord::Rotation2Y);
    qy.setName("qy");
    auto& qz = joint->updCoordinate(GimbalJoint::Coord::Rotation3Z);
    qz.setName("qz");
    model.addJoint(joint);

    auto* marker = new Marker("marker", *body, Vec3(0));
    model.addMarker(marker);

    Ellipsoid bodyGeometry(0.5, 0.1, 0.1);
    bodyGeometry.setColor(SimTK::Gray);
    PhysicalOffsetFrame* body_center = new PhysicalOffsetFrame(
            "body_center", *body, SimTK::Transform(Vec3(-0.5, 0, 0)));
    body->addComponent(body_center);
    body_center->attachGeometry(bodyGeometry.clone());

    model.finalizeConnections();

    return model;
}

class MocoMinimumDistanceConstraintPair : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoMinimumDistanceConstraintPair, Object);

public:
    OpenSim_DECLARE_PROPERTY(
            first_frame, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(
            second_frame, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(minimum_radius, double, "TODO");

    MocoMinimumDistanceConstraintPair() {
        constructProperties();
    }
    MocoMinimumDistanceConstraintPair(std::string firstFrame, 
            std::string secondFrame, double radius) 
            : MocoMinimumDistanceConstraintPair() {
        set_first_frame(firstFrame);
        set_second_frame(secondFrame);
        set_minimum_radius(radius);
    }

private:
    void constructProperties() {
        constructProperty_first_frame("");
        constructProperty_second_frame("");
        constructProperty_minimum_radius(1.0);
    }
};


class MocoMinimumDistanceConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
        MocoMinimumDistanceConstraint, MocoPathConstraint);

public:
    OpenSim_DECLARE_LIST_PROPERTY(frame_pairs,
            MocoMinimumDistanceConstraintPair, "TODO");
    MocoMinimumDistanceConstraint() {
        constructProperties();
    }

    void addFramePair(MocoMinimumDistanceConstraintPair pair) {
        append_frame_pairs(std::move(pair));
    }

protected:
    void initializeOnModelImpl(const Model& model, const MocoProblemInfo&) const 
            override {
        int nFramePairs = getProperty_frame_pairs().size();

        MocoConstraintInfo info;
        std::vector<MocoBounds> bounds;
        for (int i = 0; i < nFramePairs; ++i) {
            const auto first_frame_path = get_frame_pairs(i).get_first_frame();
            OPENSIM_THROW_IF(!model.hasComponent<Frame>(first_frame_path),
                    Exception, format("Could not find frame '%s'.", 
                            first_frame_path));
            auto& first_frame = model.getComponent<Frame>(first_frame_path);
            const auto second_frame_path = 
                    get_frame_pairs(i).get_second_frame();
            OPENSIM_THROW_IF(!model.hasComponent<Frame>(second_frame_path),
                    Exception, format("Could not find frame '%s'.", 
                            second_frame_path));
            auto& second_frame = model.getComponent<Frame>(second_frame_path);

            m_frame_pairs.emplace_back(&first_frame, &second_frame);

            bounds.emplace_back(get_frame_pairs(i).get_minimum_radius(), 
                SimTK::Infinity);
        }

        setNumEquations(nFramePairs);
        info.setBounds(bounds);
        const_cast<MocoMinimumDistanceConstraint*>(this)->setConstraintInfo(
                info);
    }

    void calcPathConstraintErrorsImpl(const SimTK::State& state, 
            SimTK::Vector& errors) const override {
        getModel().realizePosition(state);

        int iconstr = 0;
        Vec3 relative_position;
        for (const auto& frame_pair : m_frame_pairs) {
            const auto& first_frame_pos = 
                frame_pair.first->getPositionInGround(state);
            const auto& second_frame_pos =
                frame_pair.second->getPositionInGround(state);
            relative_position = second_frame_pos - first_frame_pos;

            errors[iconstr++] = relative_position.norm();
        }
        
    }
private:
    void constructProperties() {
        constructProperty_frame_pairs();
    }

    mutable std::vector<std::pair<SimTK::ReferencePtr<const Frame>, 
            SimTK::ReferencePtr<const Frame>>> m_frame_pairs;
};

int main() {

    using SimTK::Pi;


    Model model = createModel();
    model.print("3Dpendulum.osim");

    //model.setUseVisualizer(true);
    //auto state = model.initSystem();
    //state.updQ()[1] = -Pi/4;
    //model.realizePosition(state);
    //const Marker& marker = model.getComponent<Marker>("/markerset/marker");
    //std::cout << "marker loc: " << marker.getLocationInGround(state) << std::endl;

    //model.getVisualizer().show(state);


    
    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelProcessor(ModelProcessor(createModel()) | 
        ModOpAddReserves(50));

    problem.setTimeBounds(0, 0.5);
    problem.setStateInfo("/jointset/gimbal/qx/value", {-Pi/3, Pi/3}, 0);
    problem.setStateInfo(
            "/jointset/gimbal/qy/value", {-Pi/3, Pi/3}, Pi/4, -Pi/4);
    problem.setStateInfo("/jointset/gimbal/qz/value", {-Pi/3, Pi/3}, 0);
    problem.setStateInfo("/jointset/gimbal/qx/speed", {-10, 10}, 0, 0);
    problem.setStateInfo("/jointset/gimbal/qy/speed", {-10, 10}, 0, 0);
    problem.setStateInfo("/jointset/gimbal/qz/speed", {-10, 10}, 0, 0);

    auto* distance = problem.addPathConstraint<MocoMinimumDistanceConstraint>();
    MocoMinimumDistanceConstraintPair pair("/ground", "/bodyset/body", 0.1);
    distance->addFramePair(pair);

    //problem.addGoal<MocoControlGoal>();

    auto* finalMarkerGoal = problem.addGoal<MocoMarkerFinalGoal>("fina_marker");
    finalMarkerGoal->setPointName("/markerset/marker");
    finalMarkerGoal->setReferenceLocation(Vec3(0, 0.292893, 0.707107));

    //auto& solver = study.initCasADiSolver();
    //solver.set_optim_max_iterations(25);

    MocoSolution solution = study.solve().unseal();
    study.visualize(solution);


    return EXIT_SUCCESS;
}