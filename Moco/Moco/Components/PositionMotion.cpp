/* -------------------------------------------------------------------------- *
 * OpenSim Moco: PositionMotion.cpp                                           *
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

#include "PositionMotion.h"

#include "../MocoUtilities.h"

#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>

using namespace OpenSim;

class SimTKPositionMotionImplementation
        : public SimTK::Motion::Custom::Implementation {
public:
    void setFunctions(std::vector<Function*> functions) {
        m_functions = std::move(functions);
    }

    SimTK::Motion::Level getLevel(const SimTK::State&) const override {
        return SimTK::Motion::Level::Position;
    }
    void calcPrescribedPosition(
            const SimTK::State& s, int nq, SimTK::Real* q) const override {
        // std::cout << "DEBUG prescribedMotion " << nq << " " <<
        // m_functions.size() << std::endl;
        if (m_functions.size()) {
            for (int i = 0; i < nq; ++i) {
                m_funcArgs[0] = s.getTime();
                q[i] = m_functions[i]->calcValue(m_funcArgs);
                // std::cout << q[i] << " ";
            }
        }
        // std::cout << std::endl;
    }
    void calcPrescribedPositionDot(
            const SimTK::State& s, int nq, SimTK::Real* qdot) const override {
        if (m_functions.size()) {
            for (int i = 0; i < nq; ++i) {
                m_funcArgs[0] = s.getTime();
                qdot[i] = m_functions[i]->calcDerivative(
                        m_qdotDerivComponents, m_funcArgs);
            }
        }
    }
    void calcPrescribedPositionDotDot(const SimTK::State& s, int nq,
            SimTK::Real* qdotdot) const override {
        if (m_functions.size()) {
            for (int i = 0; i < nq; ++i) {
                m_funcArgs[0] = s.getTime();
                qdotdot[i] = m_functions[i]->calcDerivative(
                        m_qdotdotDerivComponents, m_funcArgs);
            }
        }
    }

private:
    std::vector<Function*> m_functions;
    mutable SimTK::Vector m_funcArgs = SimTK::Vector(1);
    static const std::vector<int> m_qdotDerivComponents;
    static const std::vector<int> m_qdotdotDerivComponents;
};

const std::vector<int>
        SimTKPositionMotionImplementation::m_qdotDerivComponents = {0};
const std::vector<int>
        SimTKPositionMotionImplementation::m_qdotdotDerivComponents = {0, 0};

class SimTKPositionMotion : public SimTK::Motion::Custom {
public:
    SimTKPositionMotion(SimTK::MobilizedBody& mobod)
            : Motion::Custom(mobod, new SimTKPositionMotionImplementation()) {}
    void setFunctions(std::vector<Function*> functions) {
        static_cast<SimTKPositionMotionImplementation&>(updImplementation())
                .setFunctions(std::move(functions));
    }
};

void PositionMotion::setPositionForCoordinate(
        const Coordinate& coord, const Function& position) {
    const auto path = coord.getAbsolutePathString();
    auto pos = std::unique_ptr<Function>(position.clone());
    pos->setName(path);
    if (get_functions().contains(path)) {
        upd_functions().set(get_functions().getIndex(path), *pos);
    } else {
        upd_functions().adoptAndAppend(pos.release());
    }
}

std::unique_ptr<PositionMotion> PositionMotion::createFromTable(
        const Model& model, const TimeSeriesTable& table,
        bool allowExtraColumns) {
    auto posmot = std::unique_ptr<PositionMotion>(new PositionMotion());
    const auto& labels = table.getColumnLabels();
    // TODO: Avoid splining extra columns.
    GCVSplineSet splines(table);
    for (auto label : labels) {
        std::string coordPath = label;
        if (endsWith(label, "/value")) {
            // This assumes that the coordinate is not named "value".
            coordPath = label.substr(0, label.find("/value"));
            const auto& coord = model.getComponent<Coordinate>(coordPath);
            const auto path = coord.getAbsolutePathString();
            posmot->setPositionForCoordinate(coord, splines.get(label));
        } else {
            OPENSIM_THROW_IF(!model.findComponent<Component>(coordPath) &&
                                     !allowExtraColumns,
                    Exception,
                    format("Column '%s' is not a coordinate.", label));
        }
    }
    return posmot;
}

void PositionMotion::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    auto& matter = system.updMatterSubsystem();
    m_motions.clear();
    for (int imb = 0; imb < matter.getNumBodies(); ++imb) {
        auto& mobod = matter.updMobilizedBody(SimTK::MobilizedBodyIndex(imb));
        m_motions.push_back(SimTKPositionMotion(mobod));
    }
}

void PositionMotion::extendRealizeTopology(SimTK::State& state) const {
    Super::extendRealizeTopology(state);
    const auto coords = getModel().getComponentList<Coordinate>();
    for (const auto& coord : coords) {
        const auto& path = coord.getAbsolutePathString();
        OPENSIM_THROW_IF(!get_functions().contains(path), Exception,
                format("No function provided for coordinate '%s'.", path));
    }

    std::map<std::pair<SimTK::MobilizedBodyIndex, int>, std::string>
            indicesToCoordName;
    for (int i = 0; i < get_functions().getSize(); ++i) {
        const auto& path = get_functions().get(i).getName();
        const auto& coord = getModel().getComponent<Coordinate>(path);
        const auto mbi = coord.getBodyIndex();
        const auto qIndex = coord.getMobilizerQIndex();
        indicesToCoordName[std::make_pair(mbi, qIndex)] = path;
    }

    auto& matter = getSystem().getMatterSubsystem();
    for (SimTK::MobilizedBodyIndex mbi(0); mbi < matter.getNumBodies(); ++mbi) {
        auto& mobod = matter.getMobilizedBody(mbi);
        std::vector<Function*> mobodFunctions;
        for (int iq = 0; iq < mobod.getNumQ(state); ++iq) {
            const auto key = std::make_pair(mbi, iq);
            // This skips over unused quaternion slots.
            if (indicesToCoordName.count(key)) {
                const auto& coordName = indicesToCoordName.at(key);
                OPENSIM_THROW_IF_FRMOBJ(!get_functions().contains(coordName),
                        Exception,
                        format("No function provided for coordinate '%s'.",
                                coordName));
                mobodFunctions.push_back(
                        const_cast<Function*>(&get_functions().get(coordName)));
            }
        }
        auto& motion = const_cast<SimTK::Motion&>(m_motions[mbi]);
        auto& customMotion = static_cast<SimTKPositionMotion&>(motion);
        customMotion.setFunctions(std::move(mobodFunctions));
    }
}
