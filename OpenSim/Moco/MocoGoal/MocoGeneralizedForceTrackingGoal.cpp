/* -------------------------------------------------------------------------- *
 * OpenSim: MocoGeneralizedForceTrackingGoal.cpp                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
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

#include "MocoGeneralizedForceTrackingGoal.h"
#include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>

using namespace OpenSim;

MocoGeneralizedForceTrackingGoal::MocoGeneralizedForceTrackingGoal() {
    constructProperties();
}

void MocoGeneralizedForceTrackingGoal::constructProperties() {
    constructProperty_reference(TableProcessor());
    constructProperty_force_paths();
    constructProperty_generalized_force_weights(MocoWeightSet());
    constructProperty_generalized_force_weights_pattern(MocoWeightSet());
    constructProperty_allow_unused_references(false);
    constructProperty_normalize_tracking_error(false);
    constructProperty_ignore_constrained_coordinates(true);
}

void MocoGeneralizedForceTrackingGoal::setWeightForGeneralizedForce(
        const std::string& name, double weight) {
    if (get_generalized_force_weights().contains(name)) {
        upd_generalized_force_weights().get(name).setWeight(weight);
    } else {
        upd_generalized_force_weights().cloneAndAppend({name, weight});
    }
}

void MocoGeneralizedForceTrackingGoal::setWeightForGeneralizedForcePattern(
        const std::string& pattern, double weight) {
    if (get_generalized_force_weights_pattern().contains(pattern)) {
        upd_generalized_force_weights_pattern().get(pattern).setWeight(weight);
    } else {
        upd_generalized_force_weights_pattern().cloneAndAppend(
                {pattern, weight});
    }
}

void MocoGeneralizedForceTrackingGoal::initializeOnModelImpl(
        const Model& model) const {

    // Get the reference data.
    TimeSeriesTable tableToUse = get_reference().process(&model);
    auto allSplines = GCVSplineSet(tableToUse);

    // Check that there are no redundant columns in the reference data.
    TableUtilities::checkNonUniqueLabels(tableToUse.getColumnLabels());

    // Create a map between coordinate paths and their indices in the system.
    SimTK::State state = model.getWorkingState();
    const auto& coordinates = model.getCoordinatesInMultibodyTreeOrder();
    std::unordered_map<std::string, int> allGeneralizedForceIndexes;
    for (int i = 0; i < static_cast<int>(coordinates.size()); ++i) {
        if (get_ignore_constrained_coordinates() && 
                coordinates[i]->isConstrained(state)) {
            continue;
        }

        std::string label = coordinates[i]->getName();
        if (coordinates[i]->getMotionType() == 
                Coordinate::MotionType::Rotational) {
            label += "_moment";
        } else if (coordinates[i]->getMotionType() == 
                Coordinate::MotionType::Translational) {
            label += "_force";
        } else if (coordinates[i]->getMotionType() == 
                Coordinate::MotionType::Coupled) {
            label += "_force";
        } else {
            OPENSIM_THROW_FRMOBJ(Exception,
                    "Expected coordinate '{}' to have Coordinate::MotionType "
                    "of Translational, Rotational, or Coupled, but it is "
                    "undefined.",
                    coordinates[i]->getName());
        }

        allGeneralizedForceIndexes[label] = i;
    }

    // Get the system indexes for specified forces.
    std::regex regex;
    for (int i = 0; i < getProperty_force_paths().size(); ++i) {
        const auto& forcePath = get_force_paths(i);
        regex = std::regex(forcePath);
        for (const auto& force : model.getComponentList<Force>()) {
            if (std::regex_match(force.getAbsolutePathString(), regex)) {
                // Check if the index already exists in the list.
                auto it = std::find(m_forceIndexes.begin(), 
                    m_forceIndexes.end(), force.getForceIndex());
                OPENSIM_THROW_IF_FRMOBJ(it != m_forceIndexes.end(), Exception,
                    "Expected unique force paths, but force at path " 
                    "'{}' was specified multiple times (possibly due to "
                    "specifying the full path and a regex pattern).", 
                    force.getAbsolutePathString());
                
                m_forceIndexes.push_back(force.getForceIndex());
            }
        }
    }

    // Add in the gravitational force index.
    const SimTK::Force::Gravity& gravity = model.getGravityForce();
    m_forceIndexes.push_back(gravity.getForceIndex());

    // Set the regex pattern weights first.
    std::map<std::string, double> weightsFromPatterns;
    for (int i = 0; i < get_generalized_force_weights_pattern().getSize(); ++i) {
        const auto& mocoWeight = get_generalized_force_weights_pattern().get(i);
        const auto& pattern = mocoWeight.getName();
        const auto regex = std::regex(pattern);
        for (const auto& kv : allGeneralizedForceIndexes) {
            if (std::regex_match(kv.first, regex)) {
                weightsFromPatterns[kv.first] = mocoWeight.getWeight();
            }
        }
    }

    // Validate the coordinate weights.
    for (int i = 0; i < get_generalized_force_weights().getSize(); ++i) {
        const auto& weightName = 
            get_generalized_force_weights().get(i).getName();
        if (allGeneralizedForceIndexes.count(weightName) == 0) {
            OPENSIM_THROW_FRMOBJ(Exception,
                    "Weight provided with name '{}' but this is "
                    "not a recognized generalized force or it is associated "
                    "with a constrained coordinate and set to be ignored.",
                    weightName);
        }
    }

    for (int iref = 0; iref < allSplines.getSize(); ++iref) {
        const auto& refName = allSplines[iref].getName();
        if (allGeneralizedForceIndexes.count(refName) == 0) {
            if (get_allow_unused_references()) { continue; }
            OPENSIM_THROW_FRMOBJ(Exception,
                    "Reference provided with name '{}' but this is "
                    "not a recognized generalized force or it is associated "
                    "with a constrained coordinate and set to be ignored.",
                    refName);
        }

        double refWeight = 1.0;
        if (get_generalized_force_weights().contains(refName)) {
            refWeight = 
                get_generalized_force_weights().get(refName).getWeight();
            OPENSIM_THROW_IF_FRMOBJ(refWeight < 0, Exception,
                    "Expected coordinate weights to be non-negative, but "
                    "received a negative weight for coordinate '{}'.",
                    refName);
        } else if (weightsFromPatterns.count(refName)) {
            refWeight = weightsFromPatterns[refName];
        }

        if (refWeight > SimTK::SignificantReal) {
            m_generalizedForceIndexes.push_back(
                    allGeneralizedForceIndexes.at(refName));
            m_generalizedForceWeights.push_back(refWeight);
            m_generalizedForceNames.push_back(refName);
            m_refsplines.cloneAndAppend(allSplines[iref]);
        } else {
            log_info("MocoGeneralizedForceTrackingGoal: Generalized force '{}' "
                     "has weight 0 (or very close to 0) and will be ignored.", 
                     refName);
        }

        // Compute normalization factors.
        if (get_normalize_tracking_error()) {
            double factor = SimTK::max(
                tableToUse.getDependentColumn(refName).abs());
            OPENSIM_THROW_IF_FRMOBJ(factor < SimTK::SignificantReal,
                    Exception,
                    "The property `normalize_tracking_error` was enabled, "
                    "but the peak magnitude of the generalized force for "
                    "coordinate is close to zero.",
                    refName);
            m_normalizationFactors.push_back(1.0 / factor);
        } else {
            m_normalizationFactors.push_back(1.0);
        }
    }

    setRequirements(1, 1, SimTK::Stage::Acceleration);
}

void MocoGeneralizedForceTrackingGoal::calcIntegrandImpl(
        const IntegrandInput& input, SimTK::Real& integrand) const {
    const auto& time = input.time;
    const auto& state = input.state;
    const auto& matter = getModel().getMatterSubsystem();

    // Get the generalized accelerations.
    getModel().realizeAcceleration(state);
    const auto& udot = state.getUDot();

    // Compute the applied body and mobility forces.
    SimTK::Vector_<SimTK::SpatialVec> appliedBodyForces(matter.getNumBodies());
    SimTK::Vector appliedMobilityForces(matter.getNumMobilities());
    getModel().calcForceContributionsSum(
        state, m_forceIndexes, appliedBodyForces, appliedMobilityForces);

    // Compute the generalized forces.
    SimTK::Vector generalizedForces(input.state.getNU(), 0.0);
    matter.calcResidualForceIgnoringConstraints(
            state, appliedMobilityForces, appliedBodyForces, udot, 
            generalizedForces);

    // TODO cache the reference force values at the mesh points, rather
    // than evaluating the spline.
    SimTK::Vector timeVec(1, time);
    integrand = 0;
    for (int iref = 0; iref < m_refsplines.getSize(); ++iref) {
        const auto& modelValue =  
                generalizedForces[m_generalizedForceIndexes[iref]];
        const auto& refValue = m_refsplines[iref].calcValue(timeVec);

        // Compute the tracking error.
        double error = m_normalizationFactors[iref] * (modelValue - refValue);

        // Compute the integrand.
        integrand += m_generalizedForceWeights[iref] * error * error;
    }
}

void MocoGeneralizedForceTrackingGoal::printDescriptionImpl() const {
    for (int i = 0; i < static_cast<int>(m_generalizedForceNames.size()); i++) {
        log_info("        generalized force: {}, weight: {}",
                m_generalizedForceNames[i],
                m_generalizedForceWeights[i]);
    }
}
