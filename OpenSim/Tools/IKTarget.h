#ifndef __IKTarget_h__
#define __IKTarget_h__
// IKTarget.h
// Authors: Ayman Habib, Peter Loan, Eran Guendelman
/* Copyright (c)  2005, Stanford University, Ayman Habib, Peter Loan, and Eran Guendelman.
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
#include "osimToolsDLL.h"
#include <OpenSim/Common/rdOptimizationTarget.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <OpenSim/Simulation/Model/AbstractMarker.h>

namespace OpenSim {

class Storage;
class IKTaskSet;

#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif

//=============================================================================
/**
 * A Class that represents an inverse kinematics optimization target. This solves the problem
 * of computing Qs that minimize the sum of the squared differences between a set of markers on a 
 * on the nominal model and passed in marker positions.
 *
 * @author Ayman Habib, Peter Loan
 * @version 1.0
 */

class OSIMTOOLS_API IKTarget : public rdOptimizationTarget
{
//==============================================================================
// DATA
//==============================================================================
private:
	Model& _model;

	// Task set contains weights and other properties affecting performance criterion
	IKTaskSet& _ikTaskSet;

	// Amount of perturbation used for derivative computation this should be an array
	static const double _perturbation;

	// internal datastructure used to map data columns to states
	Storage&	_experimentalDataStorage;

	// internal flag used for printing weighted errors at end of optimization
	bool _printPerformanceValues;

	// Marker Map information
	typedef struct
	{
		const AbstractMarker* marker;
		const AbstractBody* body;
		int experimentalColumn; // always >= 0
		double weight; // always nonzero
		// set each frame before optimization
		bool validExperimentalPosition;
		double experimentalPosition[3];
		// set each frame during optimization
		double computedPosition[3];
	} markerToSolve;

	// Coordinate Map information
	typedef struct
	{
		AbstractCoordinate* coord;
		bool prescribed; // if prescribed, we don't solve for it
		int experimentalColumn; // if task was from _file then this will be >=0
		double constantExperimentalValue; // if task was not from_file then this fixed value will be set
		double weight; // only matters for unprescribed coordinates
		// set each frame before optimization (for _unprescribedWeightedQs only)
		double experimentalValue;
	} coordinateInfo;

	// Only markers with nonzero weights are stored in this array
	Array<markerToSolve*> _markers;
	// The coordinates that are unprescribed (i.e. the ones computed by IK)
	Array<coordinateInfo*> _unprescribedQs;
	// Unprescribed coordinates with nonzero weight (the ones that appear in the performance criterion)
	Array<coordinateInfo*> _unprescribedWeightedQs;
	// The coordinates that are prescribed
	Array<coordinateInfo*> _prescribedQs;

	mutable double	_worstMarkerError;
	mutable std::string	_nameOfWorstMarker;
	mutable double	_worstCoordinateError;
	mutable std::string _nameOfWorstCoordinate;

	bool _interrupted;

//==============================================================================
// METHODS
//==============================================================================
private:
	void buildMarkerMap(const Array<std::string>& aNameArray);
	void buildCoordinateMap(const Array<std::string>& aNameArray);
	void setErrorReportingQuantities(const double& aMarkerError, const std::string& aMarkerName,
									const double& aCoordinateError, const std::string& aCoordinateName) const;
	void IKTarget::createJacobian(const SimTK::Vector &jointQs, SimTK::Matrix &J);
	void IKTarget::createPseudoInverseJacobian(const SimTK::Matrix &J, SimTK::Matrix &Jinv);
public:
	//---------------------------------------------------------------------------
	// CONSTRUCTION
	//---------------------------------------------------------------------------
	IKTarget(Model &aModel, IKTaskSet &aIKTaskSet, Storage& aExperimentalDataStorage);

	virtual ~IKTarget(void);

	//---------------------------------------------------------------------------
	// UTILITIES
	//---------------------------------------------------------------------------
	void prepareToSolve(int aIndex, double* qGuess);
	void printTasks() const;
	void printPerformance(double *x);
	void interrupt() { _interrupted = true; }
	//---------------------------------------------------------------------------
	// SET AND GET
	//---------------------------------------------------------------------------
	void getComputedMarkerLocations(Array<double> &aMarkerLocations) const;
	void getExperimentalMarkerLocations(Array<double> &aMarkerLocations) const;
	void getPrescribedCoordinateValues(Array<double>& aQValues) const;
	void getUnprescribedCoordinateNames(Array<std::string>& aNameArray);
	void getPrescribedCoordinateNames(Array<std::string>& aNameArray);
	void getOutputMarkerNames(Array<std::string>& aNameArray);
	Model& getModel() { return _model; };
	int getNumUnprescribedCoordinates() { return _unprescribedQs.getSize(); }
	int getNumOutputMarkers() { return _markers.getSize(); }
	const AbstractCoordinate *getUnprescribedCoordinate(int i) const { return _unprescribedQs.get(i)->coord; }
	//--------------------------------------------------------------------------
	// REQUIRED OPTIMIZATION TARGET METHODS
	//--------------------------------------------------------------------------
	int objectiveFunc(const SimTK::Vector &parameters, const bool new_parameters, SimTK::Real &f) const;
	int gradientFunc(const SimTK::Vector &parameters, const bool new_parameters, SimTK::Vector &gradient) const;
	int iterativeOptimization(SimTK::Vector &results);
	//--------------------------------------------------------------------------
	// DEBUG & REPORTING SUPPORT FOR GUI
	//--------------------------------------------------------------------------
	const double& getWorstMarkerError() const { return _worstMarkerError; };
	const double& getWorstCoordinateError() const { return _worstCoordinateError; };
	const std::string& getNameOfWorstMarker() const { return _nameOfWorstMarker; };
	const std::string& getNameOfWorstCoordinate() const { return _nameOfWorstCoordinate; };
};

} // namespace OpenSim

#endif
