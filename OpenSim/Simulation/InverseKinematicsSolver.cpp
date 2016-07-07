/* -------------------------------------------------------------------------- *
 *                   OpenSim:  InverseKinematicsSolver.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

#include "InverseKinematicsSolver.h"
#include "CoordinateReference.h"
#include "MarkersReference.h"
#include "OrientationsReference.h"
#include "Model/Model.h"
#include "Model/MarkerSet.h"

using namespace std;
using namespace SimTK;

namespace OpenSim {

//______________________________________________________________________________
/*
 * An implementation of the InverseKinematicsSolver 
 *
 * @param model to assemble
 */
InverseKinematicsSolver::InverseKinematicsSolver(const Model& model,
        const MarkersReference &markersReference,
        SimTK::Array_<CoordinateReference> &coordinateReferences,
        double constraintWeight) :
        InverseKinematicsSolver(model, &markersReference, nullptr,
            coordinateReferences, constraintWeight = SimTK::Infinity)
{}

InverseKinematicsSolver::InverseKinematicsSolver(const Model& model,
    const MarkersReference *markersReference,
    const OrientationsReference *orientationsReference,
    SimTK::Array_<CoordinateReference> &coordinateReferences,
    double constraintWeight ) :
        AssemblySolver(model, coordinateReferences, constraintWeight)
{
    _markersReference.reset(markersReference);
    _orientationsReference.reset(orientationsReference);

    setAuthors("Ajay Seth");
    
    if (_markersReference) {
        // Do some consistency checking for markers
        const MarkerSet &modelMarkerSet = getModel().getMarkerSet();

        if (modelMarkerSet.getSize() < 1) {
            cout << "InverseKinematicsSolver: Model has no markers!" << endl;
            throw Exception("InverseKinematicsSolver: Model has no markers!");
        }
        const SimTK::Array_<std::string> &markerNames
            = _markersReference->getNames(); // size and content as in trc file

        if (markerNames.size() < 1) {
            cout << "InverseKinematicsSolver: No markers available from data provided." << endl;
            throw Exception("InverseKinematicsSolver: No markers available from data provided.");
        }
        int index = 0, cnt = 0;
        for (unsigned int i = 0; i < markerNames.size(); i++) {
            // Check if we have this marker in the model, else ignore it
            index = modelMarkerSet.getIndex(markerNames[i], index);
            if (index >= 0) //found corresponding model
                cnt++;
        }

        if (cnt < 1) {
            cout << "InverseKinematicsSolver: Marker data does not correspond to any model markers." << endl;
            throw Exception("InverseKinematicsSolver: Marker data does not correspond to any model markers.");
        }
        if (cnt < 4)
            cout << "WARNING: InverseKinematicsSolver found only " << cnt << " markers to track." << endl;
    }
}

/* Change the weighting of a marker to take affect when assemble or track is called next. 
   Update a marker's weight by name. */
void InverseKinematicsSolver::updateMarkerWeight(const std::string &markerName, double value)
{
    const Array_<std::string> &names = _markersReference->getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), markerName);
    int index = (int)std::distance(names.begin(), p);
    updateMarkerWeight(index, value);
}

/* Update a marker's weight by its index. */
void InverseKinematicsSolver::updateMarkerWeight(int markerIndex, double value)
{
    if(markerIndex >=0 && markerIndex < _markersReference->getMarkerWeightSet().getSize()){
        //_markersReference->updMarkerWeightSet()[markerIndex].setWeight(value);
        _markerAssemblyCondition->changeMarkerWeight(SimTK::Markers::MarkerIx(markerIndex), value);
    }
    else
        throw Exception("InverseKinematicsSolver::updateMarkerWeight: invalid markerIndex.");
}

/* Update all markers weights by order in the markersReference passed in to
   construct the solver. */
void InverseKinematicsSolver::updateMarkerWeights(const SimTK::Array_<double> &weights)
{
    if(static_cast<unsigned>(_markersReference->getMarkerWeightSet().getSize()) 
       == weights.size()){
        for(unsigned int i=0; i<weights.size(); i++){
            //_markersReference->updMarkerWeightSet()[i].setWeight(weights[i]);
            _markerAssemblyCondition->changeMarkerWeight(SimTK::Markers::MarkerIx(i), weights[i]);
        }
    }
    else
        throw Exception("InverseKinematicsSolver::updateMarkerWeights: invalid size of weights.");
}

/* Compute and return the spatial location of a marker in ground. */
SimTK::Vec3 InverseKinematicsSolver::computeCurrentMarkerLocation(const std::string &markerName)
{
    const Array_<std::string> &names = _markersReference->getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), markerName);
    int index = (int)std::distance(names.begin(), p);
    return computeCurrentMarkerLocation(index);
}

SimTK::Vec3 InverseKinematicsSolver::computeCurrentMarkerLocation(int markerIndex)
{
    if(markerIndex >=0 && markerIndex < _markerAssemblyCondition->getNumMarkers()){
        return _markerAssemblyCondition->findCurrentMarkerLocation(SimTK::Markers::MarkerIx(markerIndex));
    }
    else
        throw Exception("InverseKinematicsSolver::computeCurrentMarkerLocation: invalid markerIndex.");
}

/* Compute and return the spatial locations of all markers in ground. */
void InverseKinematicsSolver::computeCurrentMarkerLocations(SimTK::Array_<SimTK::Vec3> &markerLocations)
{
    markerLocations.resize(_markerAssemblyCondition->getNumMarkers());
    for(unsigned int i=0; i<markerLocations.size(); i++)
        markerLocations[i] = _markerAssemblyCondition->findCurrentMarkerLocation(SimTK::Markers::MarkerIx(i));
}


/* Compute and return the distance error between model marker and observation. */
double InverseKinematicsSolver::computeCurrentMarkerError(const std::string &markerName)
{
    const Array_<std::string> &names = _markersReference->getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), markerName);
    int index = (int)std::distance(names.begin(), p);
    return computeCurrentMarkerError(index);
}

double InverseKinematicsSolver::computeCurrentMarkerError(int markerIndex)
{
    if(markerIndex >=0 && markerIndex < _markerAssemblyCondition->getNumMarkers()){
        return _markerAssemblyCondition->findCurrentMarkerError(SimTK::Markers::MarkerIx(markerIndex));
    }
    else
        throw Exception("InverseKinematicsSolver::computeCurrentMarkerError: invalid markerIndex.");
}

/* Compute and return the distance errors between all model markers and their observations. */
void InverseKinematicsSolver::computeCurrentMarkerErrors(SimTK::Array_<double> &markerErrors)
{
    markerErrors.resize(_markerAssemblyCondition->getNumMarkers());
    for(unsigned int i=0; i<markerErrors.size(); i++)
        markerErrors[i] = _markerAssemblyCondition->findCurrentMarkerError(SimTK::Markers::MarkerIx(i));
}


/* Compute and return the squared-distance error between model marker and observation. */
double InverseKinematicsSolver::computeCurrentSquaredMarkerError(const std::string &markerName)
{
    const Array_<std::string> &names = _markersReference->getNames();
    SimTK::Array_<const std::string>::iterator p = std::find(names.begin(), names.end(), markerName);
    int index = (int)std::distance(names.begin(), p);
    return computeCurrentSquaredMarkerError(index);
}

double InverseKinematicsSolver::computeCurrentSquaredMarkerError(int markerIndex)
{
    if(markerIndex >=0 && markerIndex < _markerAssemblyCondition->getNumMarkers()){
        return _markerAssemblyCondition->findCurrentMarkerErrorSquared(SimTK::Markers::MarkerIx(markerIndex));
    }
    else
        throw Exception("InverseKinematicsSolver::computeCurrentMarkerSquaredError: invalid markerIndex.");
}

/* Compute and return the distance errors between all model marker and observations. */
void InverseKinematicsSolver::computeCurrentSquaredMarkerErrors(SimTK::Array_<double> &markerErrors)
{
    markerErrors.resize(_markerAssemblyCondition->getNumMarkers());
    for(unsigned int i=0; i<markerErrors.size(); i++)
        markerErrors[i] = _markerAssemblyCondition->findCurrentMarkerErrorSquared(SimTK::Markers::MarkerIx(i));
}

/* Marker errors are reported in order different from tasks file or model, find name corresponding to passed in index  */
std::string InverseKinematicsSolver::getMarkerNameForIndex(int markerIndex) const
{
    return _markerAssemblyCondition->getMarkerName(SimTK::Markers::MarkerIx(markerIndex));
}


/* Internal method to convert the MarkerReferences into additional goals of the 
    of the base assembly solver, that is going to do the assembly.  */
void InverseKinematicsSolver::setupGoals(SimTK::State &s)
{
    // Setup coordinates performed by the base class
    AssemblySolver::setupGoals(s);

    setupMarkersGoal(s);

    setupOrientationsGoal(s);

    updateGoals(s);
}

void InverseKinematicsSolver::setupMarkersGoal(SimTK::State &s)
{
    // If we have no markers reference to track, then return.
    if (!_markersReference) {
        return;
    }

    // Setup markers goals
    // Get lists of all markers by names and corresponding weights from the MarkersReference
    const SimTK::Array_<SimTK::String> &markerNames = _markersReference->getNames();
    SimTK::Array_<double> markerWeights;
    _markersReference->getWeights(s, markerWeights);
    // get markers defined by the model 
    const MarkerSet &modelMarkerSet = getModel().getMarkerSet();

    // get markers with specified tasks/weights
    const Set<MarkerWeight>& mwSet = _markersReference->getMarkerWeightSet();

    // now build the Goal (AsemblyCondition) for Markers
    std::unique_ptr<SimTK::Markers> condOwner(new SimTK::Markers());
    _markerAssemblyCondition.reset(condOwner.get());

    int index = -1;
    int wIndex = -1;
    SimTK::Transform X_BF;
    //Loop through all markers in the reference
    for (unsigned int i = 0; i < markerNames.size(); ++i) {
        // Check if we have this marker in the model, else ignore it
        index = modelMarkerSet.getIndex(markerNames[i], index);
        wIndex = mwSet.getIndex(markerNames[i], wIndex);
        if ((index >= 0) && (wIndex >= 0)) {
            Marker &marker = modelMarkerSet[index];
            const SimTK::MobilizedBody& mobod =
                marker.getParentFrame().getMobilizedBody();

            X_BF = marker.getParentFrame().findTransformInBaseFrame();
            _markerAssemblyCondition->
                addMarker(marker.getName(), mobod, X_BF*marker.get_location(),
                    markerWeights[i]);
        }
    }

    // Add marker goal to the ik objective and transfer ownership of the 
    // goal (AssemblyCondition) to Assembler
    updAssembler().adoptAssemblyGoal(condOwner.release());
    // lock-in the order that the observations (markers) are in and this cannot change from frame to frame
    // and we can use an array of just the data for updating
    _markerAssemblyCondition->defineObservationOrder(markerNames);
}

void InverseKinematicsSolver::setupOrientationsGoal(SimTK::State &s)
{
    // If we have no orientations reference to track, then return.
    if (!_orientationsReference) {
        return;
    }

    // Setup orientations tracking goal
    // Get list of orientations by name  
    const SimTK::Array_<SimTK::String> &osensorNames =
        _orientationsReference->getNames();

    // If no orientations in the reference to be tracked, then no goal
    // to add and we can stop.
    if (osensorNames.size() < 1) {
        return;
    }

    std::unique_ptr<SimTK::OrientationSensors> 
        condOwner(new SimTK::OrientationSensors());
    _orientationAssemblyCondition.reset(condOwner.get());

    SimTK::Array_<double> orientationWeights;
    _orientationsReference->getWeights(s, orientationWeights);
    // get orientation sensors defined by the model 
    const auto onFrames = getModel().getComponentList<PhysicalFrame>();

    for (const auto& modelFrame : onFrames) {
        const std::string& modelFrameName = modelFrame.getName();
        auto found = std::find(osensorNames.begin(), osensorNames.end(), modelFrameName);
        if (found != osensorNames.end()) {
            int index = (int)std::distance(osensorNames.begin(), found);
            _orientationAssemblyCondition->addOSensor(modelFrameName,
                modelFrame.getMobilizedBodyIndex(),
                modelFrame.findTransformInBaseFrame().R(),
                orientationWeights[index]);
        }
    }

    // Add orientations goal to the ik objective and transfer ownership of the 
    // goal (AssemblyCondition) to Assembler
    updAssembler().adoptAssemblyGoal(condOwner.release());
    // lock-in the order that the observations (orientations) are in and this
    // cannot change from frame to frame and we can use an array of just the
    // data for updating
    _orientationAssemblyCondition->defineObservationOrder(osensorNames);
}

/* Internal method to update the time, reference values and/or their weights based
    on the state */
void InverseKinematicsSolver::updateGoals(const SimTK::State &s)
{
    // update coordinates performed by the base class
    AssemblySolver::updateGoals(s);

    // specify the marker observations to be matched
    if (_markersReference) {
        _markersReference->getValues(s, _markerValues);
        _markerAssemblyCondition->moveAllObservations(_markerValues);
    }

    // specify the orientation observations to be matched
    if (_orientationsReference) {
        _orientationsReference->getValues(s, _orientationValues);
        _orientationAssemblyCondition->moveAllObservations(_orientationValues);
    }
}

} // end of namespace OpenSim
