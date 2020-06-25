#ifndef OPENSIM_MOCOSANDBOXSHARED_H
#define OPENSIM_MOCOSANDBOXSHARED_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoSandboxShared.h                                               *
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

#include <Moco/MocoGoal/MocoGoal.h>

#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>

namespace OpenSim {

// TODO rename ContactForceTracking? ExternalForceTracking?
// TODO add function that saves a file comparing the simulated and tracked GRF.
class MocoForceTrackingCost : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoForceTrackingCost, MocoGoal);
public:
    OpenSim_DECLARE_LIST_PROPERTY(forces, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(tracked_grf_components, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(free_force_window, double, "TODO");

    MocoForceTrackingCost() {
        constructProperties();
    }
    MocoForceTrackingCost(std::string name) : MocoForceTrackingCost() {
        setName(std::move(name));
    }
protected:
    void initializeOnModelImpl(const Model& model) const override {
        m_forces.clear();
        for (int i = 0; i < getProperty_forces().size(); ++i) {
            m_forces.emplace_back(
                    model.getComponent<StationPlaneContactForce>(
                            get_forces(i)));
        }
    }

    void calcCostImpl(const GoalInput& input, double& cost) const override {
        cost = input.integral;
    }
    void calcIntegrandImpl(const IntegrandInput& input,
            SimTK::Real& integrand) const override {
        getModel().realizeVelocity(input.state);
        SimTK::Vec3 netForce(0);
        SimTK::Vec3 ref(0);
        for (const auto& force : m_forces) {
            netForce += force->calcContactForceOnStation(input.state);
        }
        SimTK::Vector timeVec(1, input.state.getTime());
        ref[0] = m_refspline_x.calcValue(timeVec);
        ref[1] = m_refspline_y.calcValue(timeVec);

        // TODO: flag to specify error power?
        double error = 0;
        if (get_tracked_grf_components() == "all") {
            error = (netForce - ref).normSqr();
        } else if (get_tracked_grf_components() == "horizontal") {
            error = abs(netForce[0] - ref[0]);
        } else if (get_tracked_grf_components() == "vertical") {
            error = abs(netForce[1] - ref[1]);
        }

        if (error < get_free_force_window()) {
            integrand += 0.0;
        } else {
            integrand += error;
        }
        
    }
private:
    void constructProperties() {
        constructProperty_forces();
        constructProperty_tracked_grf_components("all");
        constructProperty_free_force_window(0.0);
    }
    mutable std::vector<SimTK::ReferencePtr<const StationPlaneContactForce>>
        m_forces;


public: // TODO
    mutable GCVSpline m_refspline_x;
    mutable GCVSpline m_refspline_y;
};

class MocoMarkerTrackingCompCost : public MocoMarkerTrackingCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoMarkerTrackingCompCost,
            MocoMarkerTrackingCost);
public:
    /// TODO: description
    void setFreeRadius(double value) {
        set_free_radius(value);
    }

    /// TODO: description, better name
    void setTrackedMarkerComponents(std::string components) {
        set_tracked_marker_components(components);
    }
protected:

    void calcCostImpl(const CostInput& input, double& cost) const override {
        cost = input.integral;
    }
    void calcIntegrandImpl(const SimTK::State& state,
            double& integrand) const override {
        const auto& time = state.getTime();
        getModel().realizePosition(state);
        SimTK::Vector timeVec(1, time);

        for (int i = 0; i < (int)m_model_markers.size(); ++i) {
            const auto& modelValue =
                    m_model_markers[i]->getLocationInGround(state);
            SimTK::Vec3 refValue;

            // Get the markers reference index corresponding to the current
            // model marker and get the reference value.
            int refidx = m_refindices[i];
            refValue[0] = m_refsplines[3*refidx].calcValue(timeVec);
            refValue[1] = m_refsplines[3*refidx + 1].calcValue(timeVec);
            refValue[2] = m_refsplines[3*refidx + 2].calcValue(timeVec);

            // Calculate distance for specified marker position components.
            double distance = 0.0;
            if (get_tracked_marker_components().find("x") != std::string::npos) {
                distance += pow(modelValue[0] - refValue[0], 2);
            }
            if (get_tracked_marker_components().find("y") != std::string::npos) {
                distance += pow(modelValue[1] - refValue[1], 2);
            }
            if (get_tracked_marker_components().find("z") != std::string::npos) {
                distance += pow(modelValue[2] - refValue[2], 2);
            }

            // Only calculate error when distance is outside of specified free
            // radius.
            if (distance <= get_free_radius()) {
                integrand += 0.0;
            } else {
                integrand += m_marker_weights[refidx] * distance;
            }
        }
    }
private:
    OpenSim_DECLARE_PROPERTY(free_radius, double,
            "TODO");

    OpenSim_DECLARE_PROPERTY(tracked_marker_components, std::string,
            "TODO");

    void constructProperties() {
        constructProperty_free_radius(0.0);
        constructProperty_tracked_marker_components("xyz");
    };

};

} // namespace OpenSim

#endif // OPENSIM_MOCOSANDBOXSHARED_H
