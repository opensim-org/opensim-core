/* -------------------------------------------------------------------------- *
 * OpenSim Moco: AccelerationMotion.cpp                                       *
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

#include "AccelerationMotion.h"

#include <SimTKsimbody.h>

using namespace OpenSim;

class PrescribedAccelerationMotionImplementation
        : public SimTK::Motion::Custom::Implementation {
public:
    PrescribedAccelerationMotionImplementation(
            const SimTK::MobilizedBodyIndex& mobodIdx,
            const AccelerationMotion& accel)
            : m_mobodIdx(mobodIdx), m_accel(accel) {}
    SimTK::Motion::Level getLevel(const SimTK::State&) const override {
        return SimTK::Motion::Level::Acceleration;
    }
    void calcPrescribedAcceleration(
            const SimTK::State& s, int nu, SimTK::Real* udot) const override {
        std::copy_n(m_accel.getUDot(s, m_mobodIdx).getContiguousScalarData(),
                nu, udot);
    }

private:
    SimTK::MobilizedBodyIndex m_mobodIdx;
    const AccelerationMotion& m_accel;
};

class PrescribedAccelerationMotion : public SimTK::Motion::Custom {
public:
    PrescribedAccelerationMotion(
            const AccelerationMotion& accel, SimTK::MobilizedBody& mobod)
            : Motion::Custom(
                      mobod, new PrescribedAccelerationMotionImplementation(
                                     mobod, accel)) {}
};

void AccelerationMotion::setUDot(
        SimTK::State& state, const SimTK::Vector& fullUDot) const {

    OPENSIM_THROW_IF(fullUDot.size() != state.getNU(), Exception,
            "Incorrect size for fullUDot.");

    const auto& matter = getSystem().getMatterSubsystem();
    const auto& defaultSub = getSystem().getDefaultSubsystem();
    int offset = 0;
    for (int i = 0; i < matter.getNumBodies(); ++i) {
        auto& mobod = matter.getMobilizedBody(SimTK::MobilizedBodyIndex(i));
        int nu = mobod.getNumU(state);
        auto& value = defaultSub.updDiscreteVariable(state, m_dvIndices[i]);
        value.updValue<SimTK::Vector>() = fullUDot(offset, nu);
        offset += nu;
    }
}

const SimTK::Vector& AccelerationMotion::getUDot(
        const SimTK::State& state, SimTK::MobilizedBodyIndex mobodIdx) const {
    return getSystem()
            .getDefaultSubsystem()
            .getDiscreteVariable(state, m_dvIndices[mobodIdx])
            .getValue<SimTK::Vector>();
}

void AccelerationMotion::setEnabled(
        SimTK::State& state, bool enabled) const {
    for (auto& motion : m_motions) {
        if (enabled) {
            motion.enable(state);
        } else {
            motion.disable(state);
        }
    }
}

void AccelerationMotion::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    m_motions.clear();
    auto& matter = system.updMatterSubsystem();
    for (int i = 0; i < matter.getNumBodies(); ++i) {
        auto& mobod = matter.updMobilizedBody(SimTK::MobilizedBodyIndex(i));
        m_motions.push_back(PrescribedAccelerationMotion(*this, mobod));
        m_motions.back().setDisabledByDefault(true);
    }
}

void AccelerationMotion::extendRealizeTopology(SimTK::State& state) const {
    Super::extendRealizeTopology(state);
    const auto& matter = getSystem().getMatterSubsystem();
    const auto& defaultSub = getSystem().getDefaultSubsystem();
    for (int i = 0; i < matter.getNumBodies(); ++i) {
        m_dvIndices.push_back(defaultSub.allocateDiscreteVariable(state,
                SimTK::Stage::Acceleration, new SimTK::Value<SimTK::Vector>()));
    }
}
