/* -------------------------------------------------------------------------- *
 * OpenSim Moco: DeGrooteFregly2016Muscle.cpp                                 *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include "DeGrooteFregly2016Muscle.h"

#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Simulation/Model/Model.h>

#include <OpenSim/Common/FileAdapter.h>

using namespace OpenSim;

const std::string DeGrooteFregly2016Muscle::STATE_ACTIVATION_NAME("activation");

void DeGrooteFregly2016Muscle::constructProperties() {
    constructProperty_default_normalized_fiber_length(1.0);
    constructProperty_activation_time_constant(0.015);
    constructProperty_deactivation_time_constant(0.060);
    constructProperty_default_activation(0.5);

    constructProperty_active_force_width_scale(1.0);
    constructProperty_fiber_damping(0.01);
    constructProperty_tendon_strain_at_one_norm_force(0.049);
    
    constructProperty_ignore_passive_fiber_force(false);
}

void DeGrooteFregly2016Muscle::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();
    OPENSIM_THROW_IF_FRMOBJ(!getProperty_optimal_force().getValueIsDefault(),
            Exception,
            "The optimal_force property is ignored for this Force; "
            "use max_isometric_force instead.");

    SimTK_ERRCHK2_ALWAYS(get_default_normalized_fiber_length() >= 0.2,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: default_normalized_fiber_length must be >= 0.2, but it is %g.",
            getName().c_str(), get_default_normalized_fiber_length());

    SimTK_ERRCHK2_ALWAYS(get_default_normalized_fiber_length() <= 1.8,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: default_normalized_fiber_length must be <= 1.8, but it is %g.",
            getName().c_str(), get_default_normalized_fiber_length());

    SimTK_ERRCHK2_ALWAYS(get_activation_time_constant() > 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: activation_time_constant must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_activation_time_constant());

    SimTK_ERRCHK2_ALWAYS(get_deactivation_time_constant() > 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: deactivation_time_constant must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_deactivation_time_constant());

    SimTK_ERRCHK2_ALWAYS(get_default_activation() > 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: default_activation must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_default_activation());

    SimTK_ERRCHK2_ALWAYS(get_active_force_width_scale() >= 1,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: active_force_width_scale must be greater than or equal to 1.0, "
            "but it is %g.",
            getName().c_str(), get_active_force_width_scale());

    SimTK_ERRCHK2_ALWAYS(get_fiber_damping() >= 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: fiber_damping must be greater than or equal to zero, "
            "but it is %g.",
            getName().c_str(), get_fiber_damping());

    SimTK_ERRCHK2_ALWAYS(get_tendon_strain_at_one_norm_force() > 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: tendon_strain_at_one_norm_force must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_tendon_strain_at_one_norm_force());

    OPENSIM_THROW_IF_FRMOBJ(get_pennation_angle_at_optimal() != 0, Exception,
            "Non-zero 'pennation angle at optimal' not supported yet.");

    OPENSIM_THROW_IF_FRMOBJ(!get_ignore_tendon_compliance(), Exception,
            "Tendon compliance not yet supported.");

    m_maxContractionVelocityInMeters =
            get_max_contraction_velocity() * get_optimal_fiber_length();
    m_kT = log((1.0 + c3) / c1) /
           (1.0 + get_tendon_strain_at_one_norm_force() - c2);
}

void DeGrooteFregly2016Muscle::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    if (!get_ignore_activation_dynamics()) {
        addStateVariable(STATE_ACTIVATION_NAME, SimTK::Stage::Dynamics);
    }
}

void DeGrooteFregly2016Muscle::extendInitStateFromProperties(
        SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);
    if (!get_ignore_activation_dynamics()) {
        setStateVariableValue(
                s, STATE_ACTIVATION_NAME, get_default_activation());
    }
}

void DeGrooteFregly2016Muscle::extendSetPropertiesFromState(
        const SimTK::State& s) {
    Super::extendSetPropertiesFromState(s);
    if (!get_ignore_activation_dynamics()) {
        set_default_activation(getStateVariableValue(s, STATE_ACTIVATION_NAME));
    }
}

void DeGrooteFregly2016Muscle::writeTableToFile(const TimeSeriesTable& table,
        const std::string& filepath) const {
    DataAdapter::InputTables tables = {{"table", &table}};
    FileAdapter::writeFile(tables, filepath);
}

void DeGrooteFregly2016Muscle::computeStateVariableDerivatives(
        const SimTK::State& s) const {

    // On a simple hanging muscle minimum time problem, I got quicker
    // convergence using the nonlinear activation dynamics from the paper, so
    // I'm using that (below) instead of these linear dynamics.
    // const auto& tau = get_activation_time_constant();
    // const SimTK::Real activationDot = (excitation - activation) / tau;
    if (!get_ignore_activation_dynamics()) {
        const auto& activation = getActivation(s);
        const auto& excitation = getControl(s);
        static const double actTimeConst = get_activation_time_constant();
        static const double deactTimeConst = get_deactivation_time_constant();
        static const double tanhSteepness = 0.1;
        //     f = 0.5 tanh(b(e - a))
        //     z = 0.5 + 1.5a
        // da/dt = [(f + 0.5)/(tau_a * z) + (-f + 0.5)*z/tau_d] * (e - a)
        const SimTK::Real timeConstFactor = 0.5 + 1.5 * activation;
        const SimTK::Real tempAct = 1.0 / (actTimeConst * timeConstFactor);
        const SimTK::Real tempDeact = timeConstFactor / deactTimeConst;
        const SimTK::Real f =
                0.5 * tanh(tanhSteepness * (excitation - activation));
        const SimTK::Real timeConst =
                tempAct * (f + 0.5) + tempDeact * (-f + 0.5);
        const SimTK::Real derivative = timeConst * (excitation - activation);
        setStateVariableDerivativeValue(s, STATE_ACTIVATION_NAME, derivative);
    }
}

double DeGrooteFregly2016Muscle::computeActuation(const SimTK::State& s) const {
    // TODO use fiber or tendon force?
    const SimTK::Real& activation = getActivation(s);
    const SimTK::Real normFiberLength = calcNormalizedFiberLength(s);
    const SimTK::Real normFiberVelocity = calcNormalizedFiberVelocity(s);

    const SimTK::Real normFiberForce = calcNormFiberForceAlongTendon(
            activation, normFiberLength, normFiberVelocity);

    return get_max_isometric_force() * normFiberForce;
}

void DeGrooteFregly2016Muscle::calcMuscleLengthInfo(
        const SimTK::State& s, MuscleLengthInfo& mli) const {
    mli.fiberLength = calcFiberLength(s);
    mli.fiberLengthAlongTendon = mli.fiberLength; // TODO: pennation
    mli.normFiberLength = calcNormalizedFiberLength(s);

    mli.normTendonLength = calcNormalizedTendonLength(s);
    mli.tendonLength = calcTendonLength(s);
    mli.tendonStrain = mli.normTendonLength - 1.0;

    mli.pennationAngle = 0;    // TODO: pennation
    mli.cosPennationAngle = 1; // TODO: pennation
    mli.sinPennationAngle = 0; // TODO: pennation

    mli.fiberPassiveForceLengthMultiplier =
            calcPassiveForceMultiplier(mli.normFiberLength);
    mli.fiberActiveForceLengthMultiplier =
            calcActiveForceLengthMultiplier(mli.normFiberLength);
}

void DeGrooteFregly2016Muscle::computeInitialFiberEquilibrium(
        SimTK::State& s) const {
    if (get_ignore_tendon_compliance()) return;
}

DataTable DeGrooteFregly2016Muscle::exportFiberLengthCurvesToTable(
        const SimTK::Vector& normFiberLengths) const {
    SimTK::Vector def;
    const SimTK::Vector* x = nullptr;
    if (normFiberLengths.nrow()) {
        x = &normFiberLengths;
    } else {
        def = createVectorLinspace(
                200, m_minNormFiberLength, m_maxNormFiberLength);
        x = &def;
    }

    DataTable table;
    table.setColumnLabels(
            {"active_force_length_multiplier", "passive_force_multiplier"});
    SimTK::RowVector row(2);
    for (int irow = 0; irow < x->nrow(); ++irow) {
        const auto& normFiberLength = x->get(irow);
        row[0] = calcActiveForceLengthMultiplier(normFiberLength);
        row[1] = calcPassiveForceMultiplier(normFiberLength);
        table.appendRow(normFiberLength, row);
    }
    return table;
}

DataTable DeGrooteFregly2016Muscle::exportTendonForceMultiplierToTable(
        const SimTK::Vector& normTendonLengths) const {
    SimTK::Vector def;
    const SimTK::Vector* x = nullptr;
    if (normTendonLengths.nrow()) {
        x = &normTendonLengths;
    } else {
        // Evaluate the inverse of the tendon curve at y = 1.
        def = createVectorLinspace(
                200, 0.95, 1.0 + get_tendon_strain_at_one_norm_force());
        x = &def;
    }

    DataTable table;
    table.setColumnLabels({"tendon_force_multiplier"});
    SimTK::RowVector row(1);
    for (int irow = 0; irow < x->nrow(); ++irow) {
        const auto& normTendonLength = x->get(irow);
        row[0] = calcTendonForceMultiplier(normTendonLength);
        table.appendRow(normTendonLength, row);
    }
    return table;
}

DataTable DeGrooteFregly2016Muscle::exportFiberVelocityMultiplierToTable(
        const SimTK::Vector& normFiberVelocities) const {
    SimTK::Vector def;
    const SimTK::Vector* x = nullptr;
    if (normFiberVelocities.nrow()) {
        x = &normFiberVelocities;
    } else {
        def = createVectorLinspace(200, -1.1, 1.1);
        x = &def;
    }

    DataTable table;
    table.setColumnLabels({"force_velocity_multiplier"});
    SimTK::RowVector row(1);
    for (int irow = 0; irow < x->nrow(); ++irow) {
        const auto& normFiberVelocity = x->get(irow);
        row[0] = calcForceVelocityMultiplier(normFiberVelocity);
        table.appendRow(normFiberVelocity, row);
    }
    return table;
}

void DeGrooteFregly2016Muscle::printCurvesToSTOFiles(
        const std::string& directory) const {
    std::string prefix =
            directory + SimTK::Pathname::getPathSeparator() + getName();
    writeTableToFile(exportFiberLengthCurvesToTable(),
            prefix + "_fiber_length_curves.sto");
    writeTableToFile(exportFiberVelocityMultiplierToTable(),
            prefix + "_fiber_velocity_multiplier.sto");
    writeTableToFile(exportTendonForceMultiplierToTable(),
            prefix + "_tendon_force_multiplier.sto");
}

void DeGrooteFregly2016Muscle::replaceMuscles(
        Model& model, bool allowUnsupportedMuscles) {

    model.finalizeConnections();

    // Create path actuators from muscle properties and add to the model. Save
    // a list of pointers of the muscles to delete.
    std::vector<Muscle*> musclesToDelete;
    auto& muscleSet = model.updMuscles();
    for (int im = 0; im < muscleSet.getSize(); ++im) {
        auto& muscBase = muscleSet.get(im);
        if (auto musc = dynamic_cast<Millard2012EquilibriumMuscle*>(
                    &muscBase)) {
            auto* actu = new DeGrooteFregly2016Muscle();
            actu->setName(musc->getName());
            musc->setName(musc->getName() + "_delete");
            actu->setMinControl(musc->getMinControl());
            actu->setMaxControl(musc->getMaxControl());

            actu->setMaxIsometricForce(musc->getMaxIsometricForce());
            actu->setOptimalFiberLength(musc->getOptimalFiberLength());
            actu->setTendonSlackLength(musc->getTendonSlackLength());
            // TODO
            // actu->setPennationAngleAtOptimalFiberLength(
            //         musc->getPennationAngleAtOptimalFiberLength());
            actu->setMaxContractionVelocity(musc->getMaxContractionVelocity());
            actu->set_ignore_tendon_compliance(
                    musc->get_ignore_tendon_compliance());
            actu->set_ignore_activation_dynamics(
                    musc->get_ignore_activation_dynamics());

            // TODO: There is a bug in Millard2012EquilibriumMuscle where
            // the default fiber length is 0.1 by default instead of
            // optimal fiber length.
            if (!SimTK::isNumericallyEqual(
                        musc->get_default_fiber_length(), 0.1)) {
                actu->set_default_normalized_fiber_length(
                        musc->get_default_fiber_length() /
                        musc->get_optimal_fiber_length());
            }
            actu->set_default_activation(musc->get_default_activation());
            actu->set_activation_time_constant(
                    musc->get_activation_time_constant());
            actu->set_deactivation_time_constant(
                    musc->get_deactivation_time_constant());
            // TODO
            actu->set_fiber_damping(0);
            // actu->set_fiber_damping(musc->get_fiber_damping());
            actu->set_tendon_strain_at_one_norm_force(
                    musc->get_TendonForceLengthCurve()
                            .get_strain_at_one_norm_force());

            const auto& pathPointSet =
                    musc->getGeometryPath().getPathPointSet();
            auto& geomPath = actu->updGeometryPath();
            for (int ipp = 0; ipp < pathPointSet.getSize(); ++ipp) {
                auto* pathPoint = pathPointSet.get(ipp).clone();
                const auto& socketNames = pathPoint->getSocketNames();
                for (const auto& socketName : socketNames) {
                    pathPoint->updSocket(socketName)
                            .connect(pathPointSet.get(ipp)
                                             .getSocket(socketName)
                                             .getConnecteeAsObject());
                }
                geomPath.updPathPointSet().adoptAndAppend(pathPoint);
            }
            model.addForce(actu);

            // Workaround for a bug in prependComponentPathToConnecteePath().
            for (auto& comp : model.updComponentList()) {
                const auto& socketNames = comp.getSocketNames();
                for (const auto& socketName : socketNames) {
                    auto& socket = comp.updSocket(socketName);
                    auto connecteePath = socket.getConnecteePath();
                    std::string prefix = "/forceset/" + actu->getName();
                    if (startsWith(connecteePath, prefix)) {
                        connecteePath = connecteePath.substr(prefix.length());
                        socket.setConnecteePath(connecteePath);
                    }
                }
            }
            musclesToDelete.push_back(musc);

        } else {
            OPENSIM_THROW_IF(!allowUnsupportedMuscles, Exception,
                    format("Muscle '%s' of type %s is unsupported and "
                           "allowUnsupportedMuscles=false.",
                            muscBase.getName(),
                            muscBase.getConcreteClassName()));
        }
    }

    // Delete the muscles.
    for (const auto* musc : musclesToDelete) {
        int index = model.getForceSet().getIndex(musc, 0);
        OPENSIM_THROW_IF(index == -1, Exception,
                format("Muscle with name %s not found in ForceSet.",
                        musc->getName()));
        bool success = model.updForceSet().remove(index);
        OPENSIM_THROW_IF(!success, Exception,
                format("Attempt to remove muscle with "
                       "name %s was unsuccessful.",
                        musc->getName()));
    }

    model.finalizeFromProperties();
    model.finalizeConnections();
}
