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

namespace SimTK {
class Markers;
}

namespace OpenSim {

class MarkersReference;

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

    InverseKinematicsSolver(const Model &model, MarkersReference &markersReference,
                            SimTK::Array_<CoordinateReference> &coordinateReferences,
                            double constraintWeight = SimTK::Infinity);
    
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

protected:
    /** Internal method to convert the CoordinateReferences into goals of the 
        assembly solver. Subclasses can override to include other goals  
        such as point of interest matching (Marker tracking). This method is
        automatically called by assemble(). */
    void setupGoals(SimTK::State &s) override;
    /** Internal method to update the time, reference values and/or their 
        weights that define the goals, based on the provided state. */
    void updateGoals(const SimTK::State &s) override;

private:
    // The marker reference values and weightings
    MarkersReference &_markersReference;

    // Non-accessible cache of the marker values to be matched at a given state
    SimTK::Array_<SimTK::Vec3> _markerValues;

    // Markers collectively form a single assembly condition for the SimTK::Assembler
    // and the memory is managed by the Assembler
    SimTK::ReferencePtr<SimTK::Markers> _markerAssemblyCondition;

//=============================================================================
};  // END of class InverseKinematicsSolver
//=============================================================================
} // namespace

#endif // OPENSIM_INVERSE_KINEMATICS_SOLVER_H_
