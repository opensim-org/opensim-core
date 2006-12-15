// SimmModelScaler.cpp
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/ScaleSet.h>
#include "SimmModelScaler.h"
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerData.h>
#include <OpenSim/Simulation/SIMM/BodySet.h>
#include <OpenSim/Simulation/SIMM/markerSet.h>
#include "SimmFileWriter.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmModelScaler::SimmModelScaler() :
	_scalingOrder(_scalingOrderProp.getValueStrArray()),
	_measurementSetProp(PropertyObj("", SimmMeasurementSet())),
	_measurementSet((SimmMeasurementSet&)_measurementSetProp.getValueObj()),
	_scaleSetProp(PropertyObj("", ScaleSet())),
	_scaleSet((ScaleSet&)_scaleSetProp.getValueObj()),
	_markerFileName(_markerFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
	_preserveMassDist(_preserveMassDistProp.getValueBool()),
	_outputJointFileName(_outputJointFileNameProp.getValueStr()),
	_outputMuscleFileName(_outputMuscleFileNameProp.getValueStr()),
	_outputModelFileName(_outputModelFileNameProp.getValueStr()),
	_outputScaleFileName(_outputScaleFileNameProp.getValueStr()),
	_maxMarkerMovement(_maxMarkerMovementProp.getValueDbl())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmModelScaler::SimmModelScaler(DOMElement *aElement) :
   Object(aElement),
	_scalingOrder(_scalingOrderProp.getValueStrArray()),
	_measurementSetProp(PropertyObj("", SimmMeasurementSet())),
	_measurementSet((SimmMeasurementSet&)_measurementSetProp.getValueObj()),
	_scaleSetProp(PropertyObj("", ScaleSet())),
	_scaleSet((ScaleSet&)_scaleSetProp.getValueObj()),
	_markerFileName(_markerFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
	_preserveMassDist(_preserveMassDistProp.getValueBool()),
	_outputJointFileName(_outputJointFileNameProp.getValueStr()),
	_outputMuscleFileName(_outputMuscleFileNameProp.getValueStr()),
	_outputModelFileName(_outputModelFileNameProp.getValueStr()),
	_outputScaleFileName(_outputScaleFileNameProp.getValueStr()),
	_maxMarkerMovement(_maxMarkerMovementProp.getValueDbl())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmModelScaler::~SimmModelScaler()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aModelScaler SimmModelScaler to be copied.
 */
SimmModelScaler::SimmModelScaler(const SimmModelScaler &aModelScaler) :
   Object(aModelScaler),
	_scalingOrder(_scalingOrderProp.getValueStrArray()),
	_measurementSetProp(PropertyObj("", SimmMeasurementSet())),
	_measurementSet((SimmMeasurementSet&)_measurementSetProp.getValueObj()),
	_scaleSetProp(PropertyObj("", ScaleSet())),
	_scaleSet((ScaleSet&)_scaleSetProp.getValueObj()),
   _markerFileName(_markerFileNameProp.getValueStr()),
	_timeRange(_timeRangeProp.getValueDblArray()),
	_preserveMassDist(_preserveMassDistProp.getValueBool()),
	_outputJointFileName(_outputJointFileNameProp.getValueStr()),
	_outputMuscleFileName(_outputMuscleFileNameProp.getValueStr()),
	_outputModelFileName(_outputModelFileNameProp.getValueStr()),
	_outputScaleFileName(_outputScaleFileNameProp.getValueStr()),
	_maxMarkerMovement(_maxMarkerMovementProp.getValueDbl())
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
 * @return Pointer to a copy of this SimmModelScaler.
 */
Object* SimmModelScaler::copy() const
{
	SimmModelScaler *scalingParams = new SimmModelScaler(*this);
	return(scalingParams);
}

//_____________________________________________________________________________
/**
 * Copy this SimmModelScaler and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmModelScaler::SimmModelScaler(DOMElement*) in order to establish the
 * relationship of the SimmModelScaler object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmModelScaler object. Finally, the data members of the
 * copy are updated using SimmModelScaler::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmModelScaler.
 */
Object* SimmModelScaler::copy(DOMElement *aElement) const
{
	SimmModelScaler *scalingParams = new SimmModelScaler(aElement);
	*scalingParams = *this;
	scalingParams->updateFromXMLNode();
	return(scalingParams);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimmModelScaler to another.
 *
 * @param aModelScaler SimmModelScaler to be copied.
 */
void SimmModelScaler::copyData(const SimmModelScaler &aModelScaler)
{
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
	_maxMarkerMovement = aModelScaler._maxMarkerMovement;
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimmModelScaler to their null values.
 */
void SimmModelScaler::setNull()
{
	setType("SimmModelScaler");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmModelScaler::setupProperties()
{
	_scalingOrderProp.setName("scaling_order");
	Array<string> sorder("");
	_scalingOrderProp.setComment("Valid options are 'measurements', 'manualScale', possibly in sequence.");
	_scalingOrderProp.setValue(sorder);
	_propertySet.append(&_scalingOrderProp);

	_measurementSetProp.setName("SimmMeasurementSet");
	_measurementSetProp.setComment("Each SimmMeasurement consists of a 'MarkerPairSet' and a 'BodyScaleSet'");
	_propertySet.append(&_measurementSetProp);

	_scaleSetProp.setName("ScaleSet");
	_scaleSetProp.setComment("Scale factors to be used for 'manual' scaling. Used only in 'manual' scaling.");
	_propertySet.append(&_scaleSetProp);

	_markerFileNameProp.setName("marker_file");
	_markerFileNameProp.setComment("Name of marker file *.trc for static pose");
	_propertySet.append(&_markerFileNameProp);

	const double defaultTimeRange[] = {-1.0, -1.0};
	_timeRangeProp.setName("time_range");
	_timeRangeProp.setComment("Range of time in markers file to use for static pose solution");
	_timeRangeProp.setValue(2, defaultTimeRange);
	_propertySet.append(&_timeRangeProp);

	_preserveMassDistProp.setName("preserve_mass_distribution");
	_preserveMassDistProp.setComment("Whether to preserve relative mass between segments.");
	_propertySet.append(&_preserveMassDistProp);

	_outputJointFileNameProp.setName("output_joint_file");
	_outputJointFileNameProp.setComment("Name of SIMM joint file to write when done scaling.");
	_propertySet.append(&_outputJointFileNameProp);

	_outputMuscleFileNameProp.setName("output_muscle_file");
	_outputMuscleFileNameProp.setComment("Name of SIMM muscle file to write when done scaling.");
	_propertySet.append(&_outputMuscleFileNameProp);

	_outputModelFileNameProp.setName("output_model_file");
	_outputModelFileNameProp.setComment("Name of OpenSim xml file to write when done scaling.");
	_propertySet.append(&_outputModelFileNameProp);

	_outputScaleFileNameProp.setName("output_scale_file");
	_outputScaleFileNameProp.setComment("Name of scales file to write when done scaling.");
	_propertySet.append(&_outputScaleFileNameProp);

	_maxMarkerMovementProp.setName("max_marker_movement");
	_maxMarkerMovementProp.setValue(-1.0); // units of this value are the units of the marker data in the static pose (usually mm)
	_maxMarkerMovementProp.setComment("Maximum amount of movement allowed in marker data when averaging frames of the static trial.");
	_propertySet.append(&_maxMarkerMovementProp);
}

//_____________________________________________________________________________
/**
 * Register the types used by this class.
 */
void SimmModelScaler::registerTypes()
{
	Object::RegisterType(SimmMeasurement());
	//Object::RegisterType(Scale());
	SimmMeasurement::registerTypes();
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
SimmModelScaler& SimmModelScaler::operator=(const SimmModelScaler &aModelScaler)
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
bool SimmModelScaler::processModel(AbstractModel* aModel, const string& aPathToSubject, double aSubjectMass)
{
	int i;
	ScaleSet theScaleSet;
	Array<double> unity(1.0, 3);

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
				/* Load the static pose marker file, and average all the
				 * frames in the user-specified time range.
			    */
				SimmMarkerData staticPose(aPathToSubject + _markerFileName);
				staticPose.averageFrames(_maxMarkerMovement, _timeRange[0], _timeRange[1]);
				staticPose.convertToUnits(aModel->getLengthUnits());

				/* Now take and apply the measurements. */
				for (int j = 0; j < _measurementSet.getSize(); j++)
				{
					if (_measurementSet.get(j)->getApply())
					{
						double scaleFactor = 1.0;
						double modelLength = takeModelMeasurement(*aModel, *_measurementSet.get(j));
						double staticPoseLength = takeStaticPoseMeasurement(staticPose, *_measurementSet.get(j));
						if (modelLength != rdMath::NAN && staticPoseLength != rdMath::NAN)
						{
							scaleFactor = staticPoseLength / modelLength;
							_measurementSet.get(j)->applyScaleFactor(scaleFactor, theScaleSet);
							cout << "Measurement " << _measurementSet.get(j)->getName() << ": model = " << modelLength << ", static pose = " << staticPoseLength << endl;
						}
						else
						{
							cout << "___WARNING___: " << _measurementSet.get(j)->getName() << " measurement not used to scale " << aModel->getName() << endl;
						}
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
						Array<double> factors(1.0, 3);
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
				// TODO: print error
			}
		}

		/* Now scale the model. */
		aModel->scale(theScaleSet, aSubjectMass, _preserveMassDist);

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
	catch (Exception &x)
	{
		x.print(cout);
		return false;
	}

	return true;
}

//_____________________________________________________________________________
/**
 * Measure a length on a model. The length is defined by the average distance
 * between 1 or more pairs of markers, as stored in a SimmMeasurement.
 *
 * @param aModel the model to measure.
 * @param aMeasurement the measurement to take.
 * @return The measured distance.
 */
double SimmModelScaler::takeModelMeasurement(const AbstractModel& aModel, const SimmMeasurement& aMeasurement) const
{
	double length;
	const string *name1 = NULL, *name2 = NULL;
	int i, numPairs;
	AbstractDynamicsEngine& engine = aModel.getDynamicsEngine();

	/* For each pair of markers, calculate the distance between them
	 * and add it to the running total.
	 */
	for (i = 0, length = 0.0, numPairs = 0; i < aMeasurement.getNumMarkerPairs(); i++)
	{
		const SimmMarkerPair& pair = aMeasurement.getMarkerPair(i);
		pair.getMarkerNames(name1, name2);
		const AbstractMarker* marker1 = engine.getMarkerSet()->get(*name1);
		const AbstractMarker* marker2 = engine.getMarkerSet()->get(*name2);
		if (marker1 && marker2)
		{
			length += engine.calcDistance(*marker1->getBody(), marker1->getOffset(), *marker2->getBody(), marker2->getOffset());
			numPairs++;
		}
		else
		{
			if (!marker1)
				cout << "___WARNING___: marker " << *name1 << " in " << aMeasurement.getName() << " measurement not found in " << aModel.getName() << endl;
			if (!marker2)
				cout << "___WARNING___: marker " << *name2 << " in " << aMeasurement.getName() << " measurement not found in " << aModel.getName() << endl;
		}
	}

	/* Divide by the number of pairs to get the average length. */
	if (numPairs == 0)
	{
		cout << "___WARNING___: could not calculate " << aMeasurement.getName() << " measurement on " << aModel.getName() << endl;
		return rdMath::NAN;
	}
	else
	{
		return length / numPairs;
	}
}

//_____________________________________________________________________________
/**
 * Measure a length in a marker set. The length is defined by the average distance
 * between 1 or more pairs of markers, as stored in a SimmMeasurement. This
 * method takes the measurement on the first frame in the SimmMarkerData.
 *
 * @param aPose the marker cloud data to measure (first frame of data is used).
 * @param aMeasurement the measurement to take.
 * @return The measured distance.
 */
double SimmModelScaler::takeStaticPoseMeasurement(const SimmMarkerData& aPose, const SimmMeasurement& aMeasurement) const
{
	double length;
	const string *name1 = NULL, *name2 = NULL;
	int i, numPairs;

	/* For each pair of markers, calculate the distance between them
	 * and add it to the running total.
	 */
	for (i = 0, length = 0.0, numPairs = 0; i < aMeasurement.getNumMarkerPairs(); i++)
	{
		int marker1 = -1, marker2 = -1;
		const SimmMarkerPair& pair = aMeasurement.getMarkerPair(i);
		pair.getMarkerNames(name1, name2);
		const Array<string>& staticPoseMarkerNames = aPose.getMarkerNames();
		for (int j = 0; j < staticPoseMarkerNames.getSize(); j++)
		{
			if (staticPoseMarkerNames[j] == *name1)
				marker1 = j;
			if (staticPoseMarkerNames[j] == *name2)
				marker2 = j;
		}
		if (marker1 >= 0 && marker2 >= 0)
		{
			double* p1 = aPose.getFrame(0)->getMarker(marker1).get();
			double* p2 = aPose.getFrame(0)->getMarker(marker2).get();
			length += sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]) + (p2[2]-p1[2])*(p2[2]-p1[2]));
			numPairs++;
		}
		else
		{
			if (marker1 < 0)
				cout << "___WARNING___: marker " << *name1 << " in " << aMeasurement.getName() << " measurement not found in " << aPose.getFileName() << endl;
			if (marker2 < 0)
				cout << "___WARNING___: marker " << *name2 << " in " << aMeasurement.getName() << " measurement not found in " << aPose.getFileName() << endl;
		}
	}

	/* Divide by the number of pairs to get the average length. */
	if (numPairs == 0)
	{
		cout << "___WARNING___: could not calculate " << aMeasurement.getName() << " measurement on file " << aPose.getFileName() << endl;
		return rdMath::NAN;
	}
	else
	{
		return length / numPairs;
	}
}

void SimmModelScaler::peteTest() const
{
	int i;
	Array<double> factors(0.0);

	cout << "   SimmModelScaler: " << getName() << endl;
	cout << "      scaling_order: " << _scalingOrder << endl;
	cout << "      measurements (" << _measurementSet.getSize() << "):" << endl;
	for (i = 0; i < _measurementSet.getSize(); i++)
		_measurementSet.get(i)->peteTest();
	cout << "      scales (" << _scaleSet.getSize() << "):" << endl;
	for (i = 0; i < _scaleSet.getSize(); i++)
	{
		_scaleSet[i]->getScaleFactors(factors);
		cout << "      Scale:" << endl;
		cout << "         body: " << _scaleSet[i]->getSegmentName() << endl;
		cout << "         scale factors: " << factors << endl;
		cout << "         apply: " << _scaleSet[i]->getApply() << endl;
	}
	cout << "      markerFileName: " << _markerFileName << endl;
	cout << "      timeRange: " << _timeRange << endl;
	cout << "      preserveMassDist: " << _preserveMassDist << endl;
	cout << "      outputJointFile: " << _outputJointFileName << endl;
	cout << "      outputMuscleFile: " << _outputMuscleFileName << endl;
	cout << "      outputModelFile: " << _outputModelFileName << endl;
	cout << "      outputScaleFile: " << _outputScaleFileName << endl;
}
