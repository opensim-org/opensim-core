#ifndef OPENSIM_INVERSE_KINEMATICS_SOLVER_H_
#define OPENSIM_INVERSE_KINEMATICS_SOLVER_H_
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  InverseKinematicsSolver.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "AssemblySolver.h"
#include "MarkersReference.h"
#include "BufferedOrientationsReference.h"

namespace SimTK {
class Markers;
class OrientationSensors;
}

namespace OpenSim {
//=============================================================================
//=============================================================================
/**
 * Solve for the coordinates (degrees of freedom) of the model that satisfy the
 * set of constraints imposed on the model and the set of desired coordinate
 * values. The InverseKinematicsSolver provides the option to convert the
 * problem to an approximate one where the constraint violations are treated as
 * penalties to be minimized rather than strictly enforced. This can speed up
 * the solution and can be used to seed the constrained problem closer to the
 * solution.
 *
 * The InverseKinematicsSolver objective:
 * \f[
 *   min: J = sum(Wm_i*(m_i-md_i)^T*(m_i-md_i)) + sum(Wq_j*(q_j-qd_j)^2) +
 *            [Wc*sum(c_{err})^2]
 * \f]
 * where m_i and md_i are the model and desired marker locations (Vec3); q_j
 * and qd_j are model and desired joint coordinates. Wm_i and Wq_j are the
 * marker and coordinate weightings, respectively, and Wc is the weighting on
 * constraint errors. When Wc == Infinity, the second term is not included,
 * but instead q is subject to the constraint equations:
 *      \f[ c_{err} = G(q)-Go = 0 \f]
 *
 * When the model (and the number of goals) is guaranteed not to change and
 * the initial state is close to the InverseKinematics solution (e.g., from the 
 * initial assemble()), then track() is an efficient method for updating the
 * configuration to determine the small change in coordinate values, q.
 *
 * See SimTK::Assembler for more algorithmic details of the underlying solver.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API InverseKinematicsSolver: public AssemblySolver
{
//==============================================================================
// METHODS
//==============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    virtual ~InverseKinematicsSolver() {}
    // No need for copy constructor or operator
    InverseKinematicsSolver(const InverseKinematicsSolver& other)            = delete;
    InverseKinematicsSolver& operator=(const InverseKinematicsSolver& other) = delete;

    InverseKinematicsSolver(const Model& model, 
                        std::shared_ptr<MarkersReference> markersReference,
                        SimTK::Array_<CoordinateReference>& coordinateReferences,
                        double constraintWeight = SimTK::Infinity);

    InverseKinematicsSolver(const Model& model,
                        std::shared_ptr<MarkersReference> markersReference,
                        std::shared_ptr<OrientationsReference> orientationsReference,
                        SimTK::Array_<CoordinateReference> &coordinateReferences,
                        double constraintWeight = SimTK::Infinity);
    // Backward compatible constructors
    InverseKinematicsSolver(const Model& model,
            const MarkersReference& markersReference,
            SimTK::Array_<CoordinateReference>& coordinateReferences,
            double constraintWeight = SimTK::Infinity): 
        InverseKinematicsSolver(model,
                        std::make_shared<MarkersReference>(markersReference),
                        nullptr,
                        coordinateReferences, constraintWeight){};

    InverseKinematicsSolver(const Model& model,
            const MarkersReference& markersReference,
            const OrientationsReference& orientationsReference,
            SimTK::Array_<CoordinateReference>& coordinateReferences,
            double constraintWeight = SimTK::Infinity)
            : InverseKinematicsSolver(model,
                      std::make_shared<MarkersReference>(markersReference),
                      std::make_shared<OrientationsReference>(
                              orientationsReference),
                      coordinateReferences, constraintWeight){};

    /* Assemble a model configuration that meets the InverseKinematics conditions  
        (desired values and constraints) starting from an initial state that  
        does not have to satisfy the constraints. */
    //virtual void assemble(SimTK::State &s);

    /* Obtain a model configuration that meets the InverseKinematics conditions  
        (desired values and constraints) given a state that satisfies or
        is close to satisfying the constraints. Note there can be no change
        in the number of constraints or desired coordinates. Desired
        coordinate values can and should be updated between repeated calls
        to track a desired trajectory of coordinate values. */
    //virtual void track(SimTK::State &s);

    /** Return the number of markers used to solve for model coordinates.
        It is a count of the number of markers in the intersection of 
        the reference markers and model markers.
        This number is guaranteed not to change after assemble() is called
        (i.e. during subsequent calls to track()).*/
    int getNumMarkersInUse() const;

    /** Return the number of orientation sensors used to solve for model
    coordinates. It is a count of the number of orientation sensors that
    intersect the reference orientations and model reference frames with
    the same name. This number is guaranteed not to change after assemble()
    is called (i.e. during subsequent calls to track()).*/
    int getNumOrientationSensorsInUse() const;

    /** Change the weighting of a marker, given the marker's name. Takes effect
        when assemble() or track() is called next. */
    void updateMarkerWeight(const std::string &markerName, double value);
    /** Change the weighting of a marker, given the marker's index. Takes effect
        when assemble() or track() is called next. */
    void updateMarkerWeight(int markerIndex, double value);
    /** Change the weighting of all markers. Takes effect when assemble() or
        track() is called next. Marker weights are specified in the same order
        as they appear in the MarkersReference that was passed in when the
        solver was constructed. */
    void updateMarkerWeights(const SimTK::Array_<double> &weights);

    /** Change the weighting of an orientation sensor, given its name. Takes
    effect when assemble() or track() is called next. */
    void updateOrientationWeight(const std::string& orientationName, double value);
    /** Change the weighting of an orientation sensor, given its index. Takes
    effect when assemble() or track() is called next. */
    void updateOrientationWeight(int orientationIndex, double value);
    /** Change the weighting of all orientation sensors. Takes effect when
    assemble() or track() is called next. Orientation weights are specified
    in the same order as they appear in the OrientationsReference that was
    passed in when the solver was constructed. */
    void updateOrientationWeights(const SimTK::Array_<double> &weights);

    /** Compute and return a marker's spatial location in the ground frame,
        given the marker's name. */
    SimTK::Vec3 computeCurrentMarkerLocation(const std::string &markerName);
    /** Compute and return a marker's spatial location in the ground frame,
        given the marker's index. */
    SimTK::Vec3 computeCurrentMarkerLocation(int markerIndex);
    /** Compute and return the spatial locations of all markers, expressed in
        the ground frame. */
    void computeCurrentMarkerLocations(SimTK::Array_<SimTK::Vec3> &markerLocations);

    /** Compute and return the distance error between a model marker and its
        observation, given the marker's name. */
    double computeCurrentMarkerError(const std::string &markerName);
    /** Compute and return the distance error between a model marker and its
        observation, given the marker's index. */
    double computeCurrentMarkerError(int markerIndex);
    /** Compute and return the distance errors between all model markers and
        their observations. */
    void computeCurrentMarkerErrors(SimTK::Array_<double> &markerErrors);

    /** Compute and return the squared-distance error between a model marker and
        its observation, given the marker's name. This method is cheaper than
        squaring the value returned by computeCurrentMarkerError(). */
    double computeCurrentSquaredMarkerError(const std::string &markerName);
    /** Compute and return the squared-distance error between a model marker and
        its observation, given the marker's index. This method is cheaper than
        squaring the value returned by computeCurrentMarkerError(). */
    double computeCurrentSquaredMarkerError(int markerIndex);
    /** Compute and return the squared-distance errors between all model markers
        and their observations. This method is cheaper than squaring the values
        returned by computeCurrentMarkerErrors(). */
    void computeCurrentSquaredMarkerErrors(SimTK::Array_<double> &markerErrors);

    /** Marker locations and errors may be computed in an order that is different
        from tasks file or listed in the model. Return the corresponding marker
        name for an index in the list of marker locations/errors returned by the
        solver. */
    std::string getMarkerNameForIndex(int markerIndex) const;

    /** Compute and return an orientation sensor's spatial orientation in the
    ground frame, given the o-sensor's name. */
    SimTK::Rotation computeCurrentSensorOrientation(const std::string& osensorName);
    /** Compute and return an orientation sensor's spatial orientation in the
    ground frame, given the o-sensor's index. */
    SimTK::Rotation computeCurrentSensorOrientation(int osensorIndex);
    /** Compute and return the spatial orientations of all o-sensors, expressed in
    the ground frame. */
    void computeCurrentSensorOrientations(
        SimTK::Array_<SimTK::Rotation>& osensorOrientations);

    /** Compute and return the orientation error between the model orientation
    sensor and its observation, given the o-sensor's name. */
    double computeCurrentOrientationError(const std::string& osensorName);
    /** Compute and return the orientation error between the model orientation
    sensor and its observation, given the o-sensor's index. */
    double computeCurrentOrientationError(int osensorIndex);
    /** Compute all the orientation errors between the model orientation
    sensors and their observations. */
    void computeCurrentOrientationErrors(SimTK::Array_<double>& osensorErrors);

    /** Orientation sensor locations and errors may be computed in an order that
    may be different from tasks file or listed in the model. Return the 
    corresponding orientation sensor name for an index in the list of
    orientations returned by the solver. */
    std::string getOrientationSensorNameForIndex(int osensorIndex) const;
    /** indicate whether time is provided by Reference objects or driver program */
    void setAdvanceTimeFromReference(bool newValue) {
        _advanceTimeFromReference = newValue;
    };

protected:
    /** Override to include point of interest matching (Marker tracking)
        as well ad Frame orientation (OSensor) tracking.
        This method extends the set of goals used by the AssemblySolver. */
    void setupGoals(SimTK::State &s) override;
    /** Internal method to update the time, reference values and/or their 
        weights that define the goals, based on the provided state. */
    void updateGoals(SimTK::State &s) override;

private:
    /** Define and apply marker tracking goal to the assembly problem. */
    void setupMarkersGoal(SimTK::State &s);

    /** Define and apply Orientations of Frames to be tracked to the
        assembly problem. */
    void setupOrientationsGoal(SimTK::State &s);

    // The marker reference values and weightings
    std::shared_ptr<MarkersReference> _markersReference;

    // The orientation reference values and weightings
    std::shared_ptr<OrientationsReference> _orientationsReference;

    // Markers collectively form a single assembly condition for the 
    // SimTK::Assembler and the memory is managed by the Assembler
    SimTK::ReferencePtr<SimTK::Markers> _markerAssemblyCondition;

    // OrientationSensors collectively form a single assembly condition for
    // the SimTK::Assembler and the memory is managed by the Assembler
    SimTK::ReferencePtr<SimTK::OrientationSensors> _orientationAssemblyCondition;

    // internal flag indicating whether time is advanced based on live data or
    // controlled by the driver porgram (typically based on pre-recorded data).
    bool _advanceTimeFromReference{false};

//=============================================================================
};  // END of class InverseKinematicsSolver
//=============================================================================
} // namespace

#endif // OPENSIM_INVERSE_KINEMATICS_SOLVER_H_
