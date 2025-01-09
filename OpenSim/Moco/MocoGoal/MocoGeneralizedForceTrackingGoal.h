#ifndef OPENSIM_MOCOGENERALIZEDFORCETRACKINGGOAL_H
#define OPENSIM_MOCOGENERALIZEDFORCETRACKINGGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoGeneralizedForceTrackingGoal.h                                *
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

#include "MocoGoal.h"

#include <OpenSim/Simulation/TableProcessor.h>
#include <OpenSim/Moco/MocoWeightSet.h>

namespace OpenSim {

/** 
\section MocoGeneralizedForceTrackingGoal
Minimize the squared difference between a model generalized coordinate force and
a reference force trajectory, summed over the generalized coordinates forces for
which a reference is provided, and integrated over the phase. The reference can 
be provided as a file name to a STO or CSV file (or other file types for which 
there is a FileAdapter), or programmaticaly as a TimeSeriesTable via a 
TableProcessor.
Tracking problems in direct collocation perform best when tracking smooth
data, so it is recommended to filter the data in the reference you provide
to the cost.

@ingroup mocogoal */
class OSIMMOCO_API MocoGeneralizedForceTrackingGoal : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoGeneralizedForceTrackingGoal, MocoGoal);

public:
    MocoGeneralizedForceTrackingGoal();
    MocoGeneralizedForceTrackingGoal(std::string name) 
            : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoGeneralizedForceTrackingGoal(std::string name, double weight) 
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /// The table containing reference trajectories for the generalized 
    /// coordinate forces to track. The column labels should follow the format
    /// used by the OpenSim Inverse Dynamics Tool: the coordinates names with 
    /// suffixes denoting whether they are translational 
    /// (e.g. `pelvis_tx_force`) or rotational (e.g., `ankle_angle_r_moment`) 
    /// generalized forces. It is assumed that coordinates with 
    /// `Coordinate::MotionType::Coupled` are suffixed with `_force` (although 
    /// it recommend that these coordinates are ignored via  
    /// `setIgnoreConstrainedCoordinates()`). The table is not loaded until the
    /// MocoProblem initialized. 
    void setReference(TableProcessor ref) {
        set_reference(std::move(ref));
    }
    /// @copydoc setReference(TableProcessor ref)
    const TableProcessor& getReference() const { return get_reference(); }

    /// The paths to model Force%s whose body and mobility forces will be
    /// applied when computing generalized coordinate forces from inverse
    /// dynamics. Paths should be specified using either full component path 
    /// names or regular expressions, but not both for the same Force.
    void setForcePaths(const std::vector<std::string>& forcePaths) {
        for (const auto& path : forcePaths) {
            append_force_paths(path);
        }
    }
    /// @copydoc setForcePaths(const std::vector<std::string>& forcePaths)
    std::vector<std::string> getForcePaths() const {
        std::vector<std::string> paths;
        for (int i = 0; i < getProperty_force_paths().size(); ++i) {
            paths.push_back(get_force_paths(i));
        }
        return paths;
    }

    /// Set the tracking weight for the generalized force for an individual 
    /// coordinate. To remove a coordinate from the cost function, set its 
    /// weight to 0. If a weight is not specified for a coordinate, the default 
    /// weight is 1.0. If a weight is already set for the requested coordinate, 
    /// then the provided weight replaces the previous weight. Weight names 
    /// should match the column labels in the reference table (e.g., 
    /// `ankle_angle_r_moment`, `pelvis_tx_force`, etc.).
    void setWeightForGeneralizedForce(const std::string& name, double weight);

    /// Set the tracking weight for all generalized forces whose names match the
    /// regular expression pattern. Multiple pairs of patterns and weights can 
    /// be provided by calling this function multiple times. If a generalized
    /// force matches multiple patterns, the weight associated with the last
    /// pattern is used.
    void setWeightForGeneralizedForcePattern(const std::string& pattern, 
            double weight);

    /// Set the MocoWeightSet to weight the generalized coordinate forces in
    /// the cost. Replaces the weight set if it already exists.
    void setWeightSet(const MocoWeightSet& weightSet) {
        upd_generalized_force_weights() = weightSet;
    }

    /// Specify whether or not extra columns in the reference are allowed.
    /// If set true, the extra references will be ignored by the cost.
    /// If false, extra reference will cause an exception to be raised.
    void setAllowUnusedReferences(bool tf) { set_allow_unused_references(tf); }

    /// Normalize the tracking error for each generalized force by the peak 
    /// magnitude of the force in the reference tracking data. If the peak 
    /// magnitude of the reference generalized force data is close to zero, an 
    /// exception is thrown (default: false).
    void setNormalizeTrackingError(bool tf) {
        set_normalize_tracking_error(tf);
    }
    /// @copydoc setNormalizeTrackingError(bool tf)
    bool getNormalizeTrackingError() { return get_normalize_tracking_error(); }

    /// Whether or not to ignore generalized forces for coordinates that are 
    /// locked, prescribed, or coupled to other coordinates. This is based on 
    /// the value returned from `Coordinate::isConstrained()` (default: true).
    void setIgnoreConstrainedCoordinates(bool tf) {
        set_ignore_constrained_coordinates(tf);
    }
    /// @copydoc setIgnoreConstrainedCoordinates(bool tf)
    bool getIgnoreConstrainedCoordinates() const {
        return get_ignore_constrained_coordinates();
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral;
    }
    void printDescriptionImpl() const override;

private:
    void constructProperties();
    OpenSim_DECLARE_PROPERTY(reference, TableProcessor,
            "Trajectories of generalized coordiante forces to track. Column "
            "labels should be coordinate paths "
            "(e.g., /jointset/ankle_r/ankle_angle_r");
    OpenSim_DECLARE_LIST_PROPERTY(force_paths, std::string,
            "Paths to model Forces whose body and mobility forces will be "
            "applied when computing generalized coordinate forces from inverse "
            "dynamics.");
    OpenSim_DECLARE_PROPERTY(generalized_force_weights, MocoWeightSet, 
            "Set of weight objects to weight the tracking of individual "
            "generalized coordinate forces in the cost.");
    OpenSim_DECLARE_PROPERTY(generalized_force_weights_pattern, MocoWeightSet, 
            "Set weights for all generalized forces matching a regular "
            "expression.");
    OpenSim_DECLARE_PROPERTY(allow_unused_references, bool,
            "Flag to determine whether or not references contained in the "
            "reference table are allowed to be ignored by the cost.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(normalize_tracking_error, bool,
            "Normalize the tracking error for each generalized force by the "
            "peak magnitude of the force in the reference tracking data. "
            "If the peak magnitude of the reference generalized force data is "
            "close to zero, an exception is thrown (default: false).");
    OpenSim_DECLARE_PROPERTY(ignore_constrained_coordinates, bool,
            "Flag to determine whether or not to ignore generalized forces for " 
            "coordinates that are locked, prescribed, or coupled to other "
            "coordinates (default: true).");

    mutable GCVSplineSet m_refsplines;
    mutable std::vector<double> m_generalizedForceWeights;
    mutable std::vector<std::string> m_generalizedForceNames;
    mutable std::vector<int> m_generalizedForceIndexes;
    mutable std::vector<double> m_normalizationFactors;
    mutable SimTK::Array_<SimTK::ForceIndex> m_forceIndexes;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOGENERALIZEDFORCETRACKINGGOAL_H
