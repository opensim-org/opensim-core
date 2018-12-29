/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoMarkerTrackingCost.h                                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

 #include "MocoMarkerTrackingCost.h"
 #include <OpenSim/Simulation/Model/Model.h>
 #include <OpenSim/Simulation/Model/Marker.h>

 using namespace OpenSim;

 void MocoMarkerTrackingCost::initializeOnModelImpl(const Model& model) const {
    
    // Cache reference pointers to model markers.
    // TODO: When should we load a markers file?
    if (get_markers_reference().get_marker_file() != "") {
        const_cast<MocoMarkerTrackingCost*>(this)->upd_markers_reference().
                loadMarkersFile(get_markers_reference().get_marker_file());
    }
    const auto& markRefNames = get_markers_reference().getNames();
    const auto& markerSet = model.getMarkerSet();
    int iset = -1;
    for (int i = 0; i < (int)markRefNames.size(); ++i) {
        if (model.hasComponent<Marker>(markRefNames[i])) {
            const auto& m = model.getComponent<Marker>(markRefNames[i]);
            // Store a pointer to the current model marker.
            m_model_markers.emplace_back(&m);
            // Store the reference index corresponding to the current model
            // marker.
            m_refindices.push_back(i);
        } else if ((iset = markerSet.getIndex(markRefNames[i])) != -1) {
            // Allow the marker ref names to be names of markers in the
            // MarkerSet.
            m_model_markers.emplace_back(&markerSet.get(iset));
            m_refindices.push_back(i);
        } else {
            if (get_allow_unused_references()) {
                continue;
            } else {
                OPENSIM_THROW_FRMOBJ(Exception,
                    "Marker '" + markRefNames[i] + "' unrecognized by the "
                    "specified model.");
            }
        }
    }

    // Get the marker weights. The MarkersReference constructor automatically
    // sets a default value of 1.0 to each marker if not provided by the user,
    // so this is generic.
    const SimTK::State& s = model.getWorkingState();
    get_markers_reference().getWeights(s, m_marker_weights);

    // Get and flatten TimeSeriesTableVec3 to doubles and create a set of
    // reference splines, one for each component of the coordinate 
    // trajectories.
    m_refsplines = GCVSplineSet(
        get_markers_reference().getMarkerTable().flatten());
 }

 void MocoMarkerTrackingCost::calcIntegralCostImpl(const SimTK::State& state, 
        double& integrand) const {
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

         double distance = (modelValue - refValue).normSqr();

         integrand += m_marker_weights[refidx] * distance;
     }
 }
