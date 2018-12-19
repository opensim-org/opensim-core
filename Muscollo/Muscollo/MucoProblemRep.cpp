/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoProblemRep.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
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

#include "MucoProblemRep.h"
#include "MucoProblem.h"

#include <unordered_set>

using namespace OpenSim;

MucoProblemRep::MucoProblemRep(const MucoProblem& problem)
        : m_problem(&problem) {
    initialize();
}
void MucoProblemRep::initialize() {

    m_state_infos.clear();
    m_control_infos.clear();
    m_parameters.clear();
    m_costs.clear();
    m_path_constraints.clear();
    m_kinematic_constraints.clear();
    m_multiplier_infos_map.clear();

    const auto& ph0 = m_problem->getPhase(0);
    m_model = m_problem->getPhase(0).getModel();
    m_model.initSystem();

    const auto stateNames = m_model.getStateVariableNames();
    for (int i = 0; i < ph0.getProperty_state_infos().size(); ++i) {
        const auto& name = ph0.get_state_infos(i).getName();
        OPENSIM_THROW_IF(stateNames.findIndex(name) == -1, Exception,
                "State info provided for nonexistent state '" + name + "'.");
    }
    OpenSim::Array<std::string> actuNames;
    const auto modelPath = m_model.getAbsolutePath();
    for (const auto& actu : m_model.getComponentList<ScalarActuator>()) {
        actuNames.append(actu.getAbsolutePathString());
    }

    // TODO can only handle ScalarActuators?
    for (int i = 0; i < ph0.getProperty_control_infos().size(); ++i) {
        const auto& name = ph0.get_control_infos(i).getName();
        OPENSIM_THROW_IF(actuNames.findIndex(name) == -1, Exception,
                "Control info provided for nonexistent actuator '"
                        + name + "'.");
    }

    // Create internal record of state and control infos, automatically
    // populated from coordinates and actuators.
    for (int i = 0; i < ph0.getProperty_state_infos().size(); ++i) {
        const auto& name = ph0.get_state_infos(i).getName();
        m_state_infos[name] = ph0.get_state_infos(i);
    }
    for (const auto& coord : m_model.getComponentList<Coordinate>()) {
        const auto stateVarNames = coord.getStateVariableNames();
        const std::string coordValueName = stateVarNames[0];
        if (m_state_infos.count(coordValueName) == 0) {
            const auto info = MucoVariableInfo(coordValueName,
                    {coord.getRangeMin(), coord.getRangeMax()}, {}, {});
            m_state_infos[coordValueName] = info;
        }
        const std::string coordSpeedName = stateVarNames[1];
        if (m_state_infos.count(coordSpeedName) == 0) {
            const auto info = MucoVariableInfo(coordSpeedName,
                    ph0.get_default_speed_bounds(), {}, {});
            m_state_infos[coordSpeedName] = info;
        }
    }
    for (int i = 0; i < ph0.getProperty_control_infos().size(); ++i) {
        const auto& name = ph0.get_control_infos(i).getName();
        m_control_infos[name] = ph0.get_control_infos(i);
    }
    for (const auto& actu : m_model.getComponentList<ScalarActuator>()) {
        const std::string actuName = actu.getAbsolutePathString();
        if (m_control_infos.count(actuName) == 0) {
            const auto info = MucoVariableInfo(actuName,
                    {actu.getMinControl(), actu.getMaxControl()}, {}, {});
            m_control_infos[actuName] = info;
        }
    }

    m_parameters.resize(ph0.getProperty_parameters().size());
    std::unordered_set<std::string> paramNames;
    for (int i = 0; i < ph0.getProperty_parameters().size(); ++i) {
        const auto& param = ph0.get_parameters(i);
        OPENSIM_THROW_IF(param.getName().empty(), Exception,
                "All parameters must have a name.");
        OPENSIM_THROW_IF(paramNames.count(param.getName()), Exception,
                "A parameter with name '" + param.getName() +
                "' already exists.");
        paramNames.insert(param.getName());
        m_parameters[i] = std::unique_ptr<MucoParameter>(
                param.clone());
        m_parameters[i]->initializeOnModel(m_model);
    }

    m_costs.resize(ph0.getProperty_costs().size());
    std::unordered_set<std::string> costNames;
    for (int i = 0; i < ph0.getProperty_costs().size(); ++i) {
        const auto& cost = ph0.get_costs(i);
        OPENSIM_THROW_IF(cost.getName().empty(), Exception,
                "All costs must have a name.");
        OPENSIM_THROW_IF(costNames.count(cost.getName()), Exception,
                "A cost with name '" + cost.getName() + "' already exists.");
        costNames.insert(cost.getName());
        m_costs[i] = std::unique_ptr<MucoCost>(cost.clone());
        m_costs[i]->initializeOnModel(m_model);
    }

    // Get property values for constraints and Lagrange multipliers.
    const auto& kcBounds = ph0.get_kinematic_constraint_bounds();
    const MucoBounds& multBounds = ph0.get_multiplier_bounds();
    MucoInitialBounds multInitBounds(multBounds.getLower(),
            multBounds.getUpper());
    MucoFinalBounds multFinalBounds(multBounds.getLower(),
            multBounds.getUpper());
    // Get model information to loop through constraints.
    const auto& matter = m_model.getMatterSubsystem();
    const auto NC = matter.getNumConstraints();
    const auto& state = m_model.getWorkingState();
    int mp, mv, ma;
    m_num_kinematic_constraint_equations = 0;
    for (SimTK::ConstraintIndex cid(0); cid < NC; ++cid) {
        const SimTK::Constraint& constraint = matter.getConstraint(cid);
        if (!constraint.isDisabled(state)) {
            constraint.getNumConstraintEquationsInUse(state, mp, mv, ma);
            MucoKinematicConstraint kc(cid, mp, mv, ma);

            // Set the bounds for this kinematic constraint based on the
            // property.
            MucoConstraintInfo kcInfo = kc.getConstraintInfo();
            std::vector<MucoBounds> kcBoundVec(
                    kc.getConstraintInfo().getNumEquations(), kcBounds);
            kcInfo.setBounds(kcBoundVec);
            kc.setConstraintInfo(kcInfo);

            // Update number of scalar kinematic constraint equations.
            m_num_kinematic_constraint_equations +=
                    kc.getConstraintInfo().getNumEquations();

            // Append this kinematic constraint to the internal vector variable.
            // TODO: Avoid copies when the vector needs to be resized.
            m_kinematic_constraints.push_back(kc);

            // Add variable infos for all Lagrange multipliers in the problem.
            // Multipliers are only added based on the number of holonomic,
            // nonholonomic, or acceleration kinematic constraints and are *not*
            // based on the number for derivatives of holonomic or nonholonomic
            // constraint equations.
            // TODO how to name multiplier variables?
            std::vector<MucoVariableInfo> multInfos;
            for (int i = 0; i < mp; ++i) {
                MucoVariableInfo info("lambda_cid" + std::to_string(cid) +
                                "_p" + std::to_string(i),
                                multBounds, multInitBounds, multFinalBounds);
                multInfos.push_back(info);
            }
            for (int i = 0; i < mv; ++i) {
                MucoVariableInfo info("lambda_cid" + std::to_string(cid) +
                                "_v" + std::to_string(i),
                                multBounds, multInitBounds, multFinalBounds);
                multInfos.push_back(info);
            }
            for (int i = 0; i < ma; ++i) {
                MucoVariableInfo info("lambda_cid" + std::to_string(cid) +
                                "_a" + std::to_string(i),
                                multBounds, multInitBounds, multFinalBounds);
                multInfos.push_back(info);
            }
            m_multiplier_infos_map.insert({kcInfo.getName(), multInfos});
        }
    }

    m_num_path_constraint_equations = 0;
    m_path_constraints.resize(ph0.getProperty_path_constraints().size());
    std::unordered_set<std::string> pcNames;
    for (int i = 0; i < ph0.getProperty_path_constraints().size(); ++i) {
        const auto& pc = ph0.get_path_constraints(i);
        OPENSIM_THROW_IF(pc.getName().empty(), Exception,
                "All costs must have a name.");
        OPENSIM_THROW_IF(pcNames.count(pc.getName()), Exception,
                "A cost with name '" + pc.getName() + "' already exists.");
        pcNames.insert(pc.getName());
        m_path_constraints[i] = std::unique_ptr<MucoPathConstraint>(pc.clone());
        m_path_constraints[i]->
                initializeOnModel(m_model, m_num_path_constraint_equations);
        m_num_path_constraint_equations +=
                m_path_constraints[i]->getConstraintInfo().getNumEquations();
    }
}

const std::string& MucoProblemRep::getName() const {
    return m_problem->getName();
}
MucoInitialBounds MucoProblemRep::getTimeInitialBounds() const {
    return m_problem->getPhase(0).get_time_initial_bounds();
}
MucoFinalBounds MucoProblemRep::getTimeFinalBounds() const {
    return m_problem->getPhase(0).get_time_final_bounds();
}
std::vector<std::string> MucoProblemRep::createStateInfoNames() const {
    std::vector<std::string> names(m_state_infos.size());
    int i = 0;
    for (const auto& info : m_state_infos) {
        names[i] = info.first;
        ++i;
    }
    return names;
}
std::vector<std::string> MucoProblemRep::createControlInfoNames() const {
    std::vector<std::string> names(m_control_infos.size());
    int i = 0;
    for (const auto& info : m_control_infos) {
        names[i] = info.first;
        ++i;
    }
    return names;
}
std::vector<std::string> MucoProblemRep::createMultiplierInfoNames() const {
    std::vector<std::string> names;
    for (const auto& kc : m_kinematic_constraints) {
        const auto& infos = m_multiplier_infos_map.at(
                kc.getConstraintInfo().getName());
        for (const auto& info : infos) {
            names.push_back(info.getName());
        }
    }
    return names;
}
std::vector<std::string> MucoProblemRep::createKinematicConstraintNames()
const {
    std::vector<std::string> names(m_kinematic_constraints.size());
    // Kinematic constraint names are stored in the internal constraint info.
    for (int i = 0; i < (int)m_kinematic_constraints.size(); ++i) {
        names[i] = m_kinematic_constraints[i].getConstraintInfo().getName();
    }
    return names;
}
std::vector<std::string> MucoProblemRep::createParameterNames() const {
    std::vector<std::string> names(m_parameters.size());
    int i = 0;
    for (const auto& param : m_parameters) {
        names[i] = param->getName();
        ++i;
    }
    return names;
}
std::vector<std::string> MucoProblemRep::createPathConstraintNames() const {
    std::vector<std::string> names(m_path_constraints.size());
    int i = 0;
    for (const auto& pc : m_path_constraints) {
        names[i] = pc->getName();
        ++i;
    }
    return names;
}
const MucoVariableInfo& MucoProblemRep::getStateInfo(
        const std::string& name) const {
    OPENSIM_THROW_IF(m_state_infos.count(name) == 0, Exception,
            "No info available for state '" + name + "'.");
    return m_state_infos.at(name);
}
const MucoVariableInfo& MucoProblemRep::getControlInfo(
        const std::string& name) const {
    OPENSIM_THROW_IF(m_control_infos.count(name) == 0, Exception,
            "No info available for control '" + name + "'.");
    return m_control_infos.at(name);
}
const MucoParameter& MucoProblemRep::getParameter(
        const std::string& name) const {

    for (const auto& param : m_parameters) {
        if (param->getName() == name) { return *param.get(); }
    }
    OPENSIM_THROW(Exception,
            "No parameter with name '" + name + "' found.");
}
const MucoPathConstraint& MucoProblemRep::getPathConstraint(
        const std::string& name) const {

    for (const auto& pc : m_path_constraints) {
        if (pc->getName() == name) { return *pc.get(); }
    }
    OPENSIM_THROW(Exception,
            "No path constraint with name '" + name + "' found.");
}
const MucoKinematicConstraint& MucoProblemRep::getKinematicConstraint(
        const std::string& name) const {

    // Kinematic constraint names are stored in the internal constraint info.
    for (const auto& kc : m_kinematic_constraints) {
        if (kc.getConstraintInfo().getName() == name) { return kc; }
    }
    OPENSIM_THROW(Exception,
            "No kinematic constraint with name '" + name + "' found.");
}
const std::vector<MucoVariableInfo>& MucoProblemRep::getMultiplierInfos(
        const std::string& kinematicConstraintInfoName) const {

    auto search = m_multiplier_infos_map.find(kinematicConstraintInfoName);
    if (search != m_multiplier_infos_map.end()) {
        return m_multiplier_infos_map.at(kinematicConstraintInfoName);
    } else {
        OPENSIM_THROW(Exception,
                "No variable infos for kinematic constraint info with name '"
                        + kinematicConstraintInfoName + "' found.");
    }
}

void MucoProblemRep::applyParametersToModel(
        const SimTK::Vector& parameterValues) const {
    OPENSIM_THROW_IF(parameterValues.size() != (int)m_parameters.size(),
            Exception, "There are " +
                    std::to_string(m_parameters.size()) + " parameters in "
                    "this MucoProblem, but " +
                    std::to_string(parameterValues.size()) +
                    " values were provided.");
    for (int i = 0; i < (int)m_parameters.size(); ++i) {
        m_parameters[i]->applyParameterToModel(parameterValues(i));
    }
}

void MucoProblemRep::printDescription(std::ostream& stream) const {
    stream << "Costs:";
    if (m_costs.empty())
        stream << " none";
    else
        stream << " (total: " << m_costs.size() << ")";
    stream << "\n";
    for (const auto& cost : m_costs) {
        stream << "  ";
        cost->printDescription(stream);
    }

    stream << "Multibody constraints: ";
    if (m_kinematic_constraints.empty())
        stream << " none";
    else
        stream << " (total: " << m_kinematic_constraints.size() << ")";
    stream << "\n";
    for (int i = 0; i < (int)m_kinematic_constraints.size(); ++i) {
        stream << "  ";
        m_kinematic_constraints[i].getConstraintInfo().printDescription(stream);
    }

    stream << "Path constraints:";
    if (m_path_constraints.empty())
        stream << " none";
    else
        stream << " (total: " << m_path_constraints.size() << ")";
    stream << "\n";
    for (const auto& pc : m_path_constraints) {
        stream << "  ";
        pc->getConstraintInfo().printDescription(stream);
    }

    stream << "States:";
    if (m_state_infos.empty())
        stream << " none";
    else
        stream << " (total: " << m_state_infos.size() << ")";
    stream << "\n";
    // TODO want to loop through the model's state variables and controls, not
    // just the infos.
    for (const auto& info : m_state_infos) {
        stream << "  ";
        info.second.printDescription(stream);
    }

    stream << "Controls:";
    if (m_control_infos.empty())
        stream << " none";
    else
        stream << " (total: " << m_control_infos.size() << "):";
    stream << "\n";
    for (const auto& info : m_control_infos) {
        stream << "  ";
        info.second.printDescription(stream);
    }

    stream << "Parameters:";
    if (m_parameters.empty())
        stream << " none";
    else
        stream << " (total: " << m_parameters.size() << "):";
    stream << "\n";
    for (const auto& param : m_parameters) {
        stream << "  ";
        param->printDescription(stream);
    }

    stream.flush();
}


