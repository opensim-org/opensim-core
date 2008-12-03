// ModelScaler.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/ScaleSet.h>
#include "ModelScaler.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Utilities/simmFileWriterDll/SimmFileWriter.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ModelScaler::ModelScaler() :
	_apply(_applyProp.getValueBool()),
	_scalingOrder(_scalingOrderProp.getValueStrArray()),
	_measurementSetProp(PropertyObj("", MeasurementSet())),
	_measurementSet((MeasurementSet&)_measurementSetProp.getValueObj()),
	_scaleSetProp(PropertyObj("", ScaleSet())),
	_scaleSet((ScaleSet&)_scaleSetProp.getValueObj()),
	_markerFileName(_markerFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
	_preserveMassDist(_preserveMassDistProp.getValueBool()),
	_outputJointFileName(_outputJointFileNameProp.getValueStr()),
	_outputMuscleFileName(_outputMuscleFileNameProp.getValueStr()),
	_outputModelFileName(_outputModelFileNameProp.getValueStr()),
	_outputScaleFileName(_outputScaleFileNameProp.getValueStr())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
ModelScaler::~ModelScaler()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aModelScaler ModelScaler to be copied.
 */
ModelScaler::ModelScaler(const ModelScaler &aModelScaler) :
   Object(aModelScaler),
	_apply(_applyProp.getValueBool()),
	_scalingOrder(_scalingOrderProp.getValueStrArray()),
	_measurementSetProp(PropertyObj("", MeasurementSet())),
	_measurementSet((MeasurementSet&)_measurementSetProp.getValueObj()),
	_scaleSetProp(PropertyObj("", ScaleSet())),
	_scaleSet((ScaleSet&)_scaleSetProp.getValueObj()),
   _markerFileName(_markerFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
	_preserveMassDist(_preserveMassDistProp.getValueBool()),
	_outputJointFileName(_outputJointFileNameProp.getValueStr()),
	_outputMuscleFileName(_outputMuscleFileNameProp.getValueStr()),
	_outputModelFileName(_outputModelFileNameProp.getValueStr()),
	_outputScaleFileName(_outputScaleFileNameProp.getValueStr())
{
	setNull();
	setupProperties();
	copyData(aModelScaler);
}

//_____________________________________________________________________________
/**
 * Copy this scaling params and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this ModelScaler.
 */
Object* ModelScaler::copy() const
{
	ModelScaler *scalingParams = new ModelScaler(*this);
	return(scalingParams);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one ModelScaler to another.
 *
 * @param aModelScaler ModelScaler to be copied.
 */
void ModelScaler::copyData(const ModelScaler &aModelScaler)
{
	_apply = aModelScaler._apply;
	_scalingOrder = aModelScaler._scalingOrder;
	_measurementSet = aModelScaler._measurementSet;
	_scaleSet = aModelScaler._scaleSet;
	_markerFileName = aModelScaler._markerFileName;
	_timeRange = aModelScaler._timeRange;
	_preserveMassDist = aModelScaler._preserveMassDist;
	_outputJointFileName = aModelScaler._outputJointFileName;
	_outputMuscleFileName = aModelScaler._outputMuscleFileName;
	_outputModelFileName = aModelScaler._outputModelFileName;
	_outputScaleFileName = aModelScaler._outputScaleFileName;
	_printResultFiles = aModelScaler._printResultFiles;
}

//_____________________________________________________________________________
/**
 * Set the data members of this ModelScaler to their null values.
 */
void ModelScaler::setNull()
{
	setType("ModelScaler");

	_apply = true;

	_printResultFiles = true;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ModelScaler::setupProperties()
{
	_applyProp.setComment("Whether or not to use the model scaler during scale");
	_applyProp.setName("apply");
	_propertySet.append(&_applyProp);

	_scalingOrderProp.setComment("Specifies the scaling method and order. "
		"Valid options are 'measurements', 'manualScale', singly or both in any sequence.");
	_scalingOrderProp.setName("scaling_order");
	Array<string> sorder("");
	_scalingOrderProp.setValue(sorder);
	_propertySet.append(&_scalingOrderProp);

	_measurementSetProp.setComment("Specifies the measurements by which body segments are to be scaled.");
	_measurementSetProp.setName("MeasurementSet");
	_propertySet.append(&_measurementSetProp);

	_scaleSetProp.setComment("Scale factors to be used for manual scaling.");
	_scaleSetProp.setName("ScaleSet");
	_propertySet.append(&_scaleSetProp);

	_markerFileNameProp.setComment("TRC file (.trc) containing the marker positions used for measurement-based scaling. "
		"This is usually a static trial, but doesn't need to be.  The marker-pair distances are computed for each "
		"time step in the TRC file and averaged across the time range.");
	_markerFileNameProp.setName("marker_file");
	_propertySet.append(&_markerFileNameProp);

	_timeRangeProp.setComment("Time range over which to average marker-pair distances in the marker file (.trc) for "
		"measurement-based scaling.");
	const double defaultTimeRange[] = {-1.0, -1.0};
	_timeRangeProp.setName("time_range");
	_timeRangeProp.setValue(2, defaultTimeRange);
	_timeRangeProp.setAllowableArraySize(2);
	_propertySet.append(&_timeRangeProp);

	_preserveMassDistProp.setComment("Flag (true or false) indicating whether or not to preserve relative mass between segments.");
	_preserveMassDistProp.setName("preserve_mass_distribution");
	_propertySet.append(&_preserveMassDistProp);

	_outputJointFileNameProp.setComment("Name of SIMM joint file to write when done scaling. "
		"If not specified, a file will not be written.");
	_outputJointFileNameProp.setName("output_joint_file");
	_propertySet.append(&_outputJointFileNameProp);

	_outputMuscleFileNameProp.setComment("Name of SIMM muscle file to write when done scaling. "
		"If not specified, a file will not be written.");
	_outputMuscleFileNameProp.setName("output_muscle_file");
	_propertySet.append(&_outputMuscleFileNameProp);

	_outputModelFileNameProp.setComment("Name of OpenSim model file (.osim) to write when done scaling.");
	_outputModelFileNameProp.setName("output_model_file");
	_propertySet.append(&_outputModelFileNameProp);

	_outputScaleFileNameProp.setComment("Name of file to write containing the scale factors that were applied to the unscaled model (optional).");
	_outputScaleFileNameProp.setName("output_scale_file");
	_propertySet.append(&_outputScaleFileNameProp);
}


//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
ModelScaler& ModelScaler::operator=(const ModelScaler &aModelScaler)
{
	// BASE CLASS
	Object::operator=(aModelScaler);

	copyData(aModelScaler);

	return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * This method scales a model based on user-specified XYZ body scale factors
 * and/or a set of marker-to-marker distance measurements.
 *
 * @param aModel the model to scale.
 * @param aSubjectMass the final mass of the model after scaling.
 * @return Whether the scaling process was successful or not.
 */
bool ModelScaler::processModel(Model* aModel, const string& aPathToSubject, double aSubjectMass)
{
	if(!getApply()) return false;

	int i;
	ScaleSet theScaleSet;
	Vec3 unity(1.0);

	cout << endl << "Step 2: Scaling generic model" << endl;

	const BodySet* bodySet = aModel->getDynamicsEngine().getBodySet();

	/* Make a scale set with an Scale for each body.
	 * Initialize all factors to 1.0.
	 */
	for (i = 0; i < bodySet->getSize(); i++)
	{
		Scale* bodyScale = new Scale();
		bodyScale->setSegmentName(bodySet->get(i)->getName());
		bodyScale->setScaleFactors(unity);
		bodyScale->setApply(true);
		theScaleSet.append(bodyScale);
	}

	try
	{
		/* Make adjustments to theScaleSet, in the user-specified order. */
		for (i = 0; i < _scalingOrder.getSize(); i++)
		{
			/* For measurements, measure the distance between a pair of markers
			 * in the model, and in the static pose. The latter divided by the
			 * former is the scale factor. Put that scale factor in theScaleSet,
			 * using the body/axis names specified in the measurement to
			 * determine in what place[s] to put the factor.
			 */
			if (_scalingOrder[i] == "measurements")
			{
				/* Load the static pose marker file, and convert units.
			    */
				MarkerData *markerData = 0;
				if(!_markerFileName.empty() && _markerFileName!=PropertyStr::getDefaultStr()) {
					markerData = new MarkerData(aPathToSubject + _markerFileName);
					markerData->convertToUnits(aModel->getLengthUnits());
				}

				/* Now take and apply the measurements. */
				for (int j = 0; j < _measurementSet.getSize(); j++)
				{
					if (_measurementSet.get(j)->getApply())
					{
						if(!markerData)
							throw Exception("ModelScaler.processModel: ERROR- "+_markerFileNameProp.getName()+
											    " not set but measurements are used",__FILE__,__LINE__);
						double scaleFactor = computeMeasurementScaleFactor(*aModel, *markerData, *_measurementSet.get(j));
						if (!rdMath::isNAN(scaleFactor))
							_measurementSet.get(j)->applyScaleFactor(scaleFactor, theScaleSet);
						else
							cout << "___WARNING___: " << _measurementSet.get(j)->getName() << " measurement not used to scale " << aModel->getName() << endl;
					}
				}
			}
			/* For manual scales, just copy the XYZ scale factors from
		  	 * the manual scale into theScaleSet.
	  		 */
			else if (_scalingOrder[i] == "manualScale")
			{
				for (int j = 0; j < _scaleSet.getSize(); j++)
				{
					if (_scaleSet[j]->getApply())
					{
						const string& bodyName = _scaleSet[j]->getSegmentName();
						Vec3 factors(1.0);
						_scaleSet[j]->getScaleFactors(factors);
						for (int k = 0; k < theScaleSet.getSize(); k++)
						{
							if (theScaleSet[k]->getSegmentName() == bodyName)
								theScaleSet[k]->setScaleFactors(factors);
						}
					}
				}
			}
			else
			{
				throw Exception("ModelScaler: ERR- Unrecognized string '"+_scalingOrder[i]+"' in "+_scalingOrderProp.getName()+" property (expecting 'measurements' or 'manualScale').",__FILE__,__LINE__);
			}
		}

		/* Now scale the model. */
		aModel->scale(theScaleSet, aSubjectMass, _preserveMassDist);

		if(_printResultFiles) {
			/* Write output files, if names specified by the user. */
			SimmFileWriter *sfw = new SimmFileWriter(aModel);
			if (sfw)
			{
				if (!_outputJointFileNameProp.getUseDefault())
					sfw->writeJointFile(_outputJointFileName);

				if (!_outputMuscleFileNameProp.getUseDefault())
					sfw->writeMuscleFile(_outputMuscleFileName);

				delete sfw;
			}

			if (!_outputModelFileNameProp.getUseDefault())
			{
				if (aModel->print(_outputModelFileName))
					cout << "Wrote model file " << _outputModelFileName << " from model " << aModel->getName() << endl;
			}

			if (!_outputScaleFileNameProp.getUseDefault())
			{
				if (theScaleSet.print(_outputScaleFileName))
					cout << "Wrote scale file " << _outputScaleFileName << " for model " << aModel->getName() << endl;
			}
		}
	}
	catch (Exception &x)
	{
		x.print(cout);
		return false;
	}

	return true;
}

//_____________________________________________________________________________
/**
 * For measurement based scaling, we average the scale factors across the different marker pairs used.
 * For each marker pair, the scale factor is computed by dividing the average distance between the pair 
 * in the experimental marker data by the distance between the pair on the model.
 */
double ModelScaler::computeMeasurementScaleFactor(const Model& aModel, const MarkerData& aMarkerData, const Measurement& aMeasurement) const
{
	double scaleFactor = 0;
	cout << "Measurement '" << aMeasurement.getName() << "'" << endl;
	if(aMeasurement.getNumMarkerPairs()==0) return rdMath::getNAN();
	for(int i=0; i<aMeasurement.getNumMarkerPairs(); i++) {
		const MarkerPair& pair = aMeasurement.getMarkerPair(i);
		string name1, name2;
		pair.getMarkerNames(name1, name2);
		double modelLength = takeModelMeasurement(aModel, name1, name2, aMeasurement.getName());
		double experimentalLength = takeExperimentalMarkerMeasurement(aMarkerData, name1, name2, aMeasurement.getName());
		if(rdMath::isNAN(modelLength) || rdMath::isNAN(experimentalLength)) return rdMath::getNAN();
		cout << "\tpair " << i << " (" << name1 << ", " << name2 << "): model = " << modelLength << ", experimental = " << experimentalLength << endl;
		scaleFactor += experimentalLength / modelLength;
	}
	scaleFactor /= aMeasurement.getNumMarkerPairs();
	cout << "\toverall scale factor = " << scaleFactor << endl;
	return scaleFactor;
}

//_____________________________________________________________________________
/**
 * Measure the distance between two model markers.
 *
 * @return The measured distance.
 */
double ModelScaler::takeModelMeasurement(const Model& aModel, const string& aName1, const string& aName2, const string& aMeasurementName) const
{
	AbstractDynamicsEngine& engine = aModel.getDynamicsEngine();
	const AbstractMarker* marker1 = engine.getMarkerSet()->get(aName1);
	const AbstractMarker* marker2 = engine.getMarkerSet()->get(aName2);

	if (marker1 && marker2) {
		return engine.calcDistance(*marker1->getBody(), marker1->getOffset(), *marker2->getBody(), marker2->getOffset());
	} else {
		if (!marker1)
			cout << "___WARNING___: marker " << aName1 << " in " << aMeasurementName << " measurement not found in " << aModel.getName() << endl;
		if (!marker2)
			cout << "___WARNING___: marker " << aName2 << " in " << aMeasurementName << " measurement not found in " << aModel.getName() << endl;
		return rdMath::getNAN();
	}
}

//_____________________________________________________________________________
/**
 * Measure the average distance between a marker pair in an experimental marker data.
 */
double ModelScaler::takeExperimentalMarkerMeasurement(const MarkerData& aMarkerData, const string& aName1, const string& aName2, const string& aMeasurementName) const
{
	const Array<string>& experimentalMarkerNames = aMarkerData.getMarkerNames();
	int marker1 = experimentalMarkerNames.findIndex(aName1);
	int marker2 = experimentalMarkerNames.findIndex(aName2);
	if (marker1 >= 0 && marker2 >= 0) {
		int startIndex, endIndex;
		aMarkerData.findFrameRange(_timeRange[0], _timeRange[1], startIndex, endIndex);
		double length = 0;
		for(int i=startIndex; i<=endIndex; i++) {
			Vec3& p1 = aMarkerData.getFrame(i)->getMarker(marker1).get();
			Vec3& p2 = aMarkerData.getFrame(i)->getMarker(marker2).get();
			length += (p2 - p1).norm();
		}
		return length/(endIndex-startIndex+1);
	} else {
		if (marker1 < 0)
			cout << "___WARNING___: marker " << aName1 << " in " << aMeasurementName << " measurement not found in " << aMarkerData.getFileName() << endl;
		if (marker2 < 0)
			cout << "___WARNING___: marker " << aName2 << " in " << aMeasurementName << " measurement not found in " << aMarkerData.getFileName() << endl;
		return rdMath::getNAN();
	}
}
