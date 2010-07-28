/* InverseKinematicsSolver.cpp 
* Author: Ajay Seth 
* Copyright (c)  2006 Stanford University
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "InverseKinematicsSolver.h"
#include "CoordinateReference.h"
#include "MarkersReference.h"
#include "Model/Model.h"
#include "Model/MarkerSet.h"

using namespace std;
using namespace SimTK;

namespace OpenSim {

//______________________________________________________________________________
/**
 * An implementation of the InverseKinematicsSolver 
 *
 * @param model to assemble
 */
InverseKinematicsSolver::InverseKinematicsSolver(const Model &model, MarkersReference &markersReference,
							SimTK::Array_<CoordinateReference> &coordinateReferences,
							double constraintWeight) : AssemblySolver(model, coordinateReferences, constraintWeight),
							_markersReference(markersReference)	
{
	// Base AssemblySolver takes care of creating the underlying _assembler and setting up CoordinateReferences;
	_markerAssemblyCondition = NULL;

	// Do some consistency checking for markers
	const MarkerSet &modelMarkerSet = _model.getMarkerSet();

	if(modelMarkerSet.getSize() < 1)
		throw Exception("InverseKinematicsSolver: Model has no markers!");
	
	
	const SimTK::Array_<std::string> &markerNames = _markersReference.getNames();

	if(markerNames.size() < 1)
		throw Exception("InverseKinematicsSolver: No markers available from data provided.");

	int index=0, cnt=0;
	for(unsigned int i=0; i < markerNames.size(); i++) {
		// Check if we have this marker in the model, else ignore it
		index = modelMarkerSet.getIndex(markerNames[i], index);
		if(index >= 0) //found corresponding model
			cnt++;
	}

	if(cnt < 1)
		throw Exception("InverseKinematicsSolver: Marker data does not correspond to any model markers.");

	if(cnt < 4)
		cout << "WARNING: InverseKinematicsSolver found only " << cnt << " markers to track." << endl;

}

/** Internal method to convert the MarkerReferences into additional goals of the 
	of the base assembly solver, that is going to do the assembly.  */
void InverseKinematicsSolver::setupGoals(SimTK::State &s)
{
	// Setup coordinates performed by the base class
	AssemblySolver::setupGoals(s);

	_markerAssemblyCondition = new SimTK::Markers();

	// Setup markers goals
	// Get lists of all markers by names and corresponding weights from the MarkersReference
	const SimTK::Array_<std::string> &markerNames = _markersReference.getNames();
	SimTK::Array_<double> markerWeights;  
	_markersReference.getWeights(s, markerWeights);
	// get markers defined by the model 
	const MarkerSet &modelMarkerSet = _model.getMarkerSet();
	
	int index = 0;
	//Loop through all markers in the reference
	for(unsigned int i=0; i < markerNames.size(); i++){
		// Check if we have this marker in the model, else ignore it
		index = modelMarkerSet.getIndex(markerNames[i], index);
		if(index >= 0){
			Marker &marker = modelMarkerSet[index];
			const SimTK::MobilizedBody &mobod = _model.getMatterSubsystem().getMobilizedBody(marker.getBody().getIndex());
			_markerAssemblyCondition->addMarker(marker.getName(), mobod, marker.getOffset(), markerWeights[i]);
			cout << "IKSolver Marker: " << markerNames[i] << " " << marker.getName() << "  weight: " << markerWeights[i] << endl;
		}
	}

	// Add marker goal to the ik objective
	_assembler->adoptAssemblyGoal(_markerAssemblyCondition);
	// lock-in the order that the observations (markers) are in and this cannot change from frame to frame
	// and we can use an array of just the data for updating
	_markerAssemblyCondition->defineObservationOrder(markerNames);

	updateGoals(s);
}

/** Internal method to update the time, reference values and/or their weights based
    on the state */
void InverseKinematicsSolver::updateGoals(const SimTK::State &s)
{
	// update coordinates performed by the base class
	AssemblySolver::updateGoals(s);

	// specify the (initial) observations to be matched
	_markersReference.getValues(s, _markerValues);
	_markerAssemblyCondition->moveAllObservations(_markerValues);
}

} // end of namespace OpenSim