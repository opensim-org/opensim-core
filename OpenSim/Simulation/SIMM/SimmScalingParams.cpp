// SimmScalingParams.cpp
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
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
#include "SimmModel.h"
#include "SimmSubject.h"
#include "SimmScalingParams.h"
#include "SimmMarkerData.h"


//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmScalingParams::SimmScalingParams() :
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
}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmScalingParams::SimmScalingParams(DOMElement *aElement) :
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
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmScalingParams::~SimmScalingParams()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aScalingParams SimmScalingParams to be copied.
 */
SimmScalingParams::SimmScalingParams(const SimmScalingParams &aScalingParams) :
   Object(aScalingParams),
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
	setupProperties();
	copyData(aScalingParams);
}
//_____________________________________________________________________________
/**
 * Copy this scaling params and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmScalingParams.
 */
Object* SimmScalingParams::copy() const
{
	SimmScalingParams *scalingParams = new SimmScalingParams(*this);
	return(scalingParams);
}
//_____________________________________________________________________________
/**
 * Copy this SimmScalingParams and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmScalingParams::SimmScalingParams(DOMElement*) in order to establish the
 * relationship of the SimmScalingParams object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmScalingParams object. Finally, the data members of the
 * copy are updated using SimmScalingParams::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmScalingParams.
 */
Object* SimmScalingParams::copy(DOMElement *aElement) const
{
	SimmScalingParams *scalingParams = new SimmScalingParams(aElement);
	*scalingParams = *this;
	scalingParams->updateFromXMLNode();
	return(scalingParams);
}

void SimmScalingParams::copyData(const SimmScalingParams &aScalingParams)
{
	_scalingOrder = aScalingParams._scalingOrder;
	_measurementSet = aScalingParams._measurementSet;
	_scaleSet = aScalingParams._scaleSet;
	_markerFileName = aScalingParams._markerFileName;
	_timeRange = aScalingParams._timeRange;
	_preserveMassDist = aScalingParams._preserveMassDist;
	_outputJointFileName = aScalingParams._outputJointFileName;
	_outputMuscleFileName = aScalingParams._outputMuscleFileName;
	_outputModelFileName = aScalingParams._outputModelFileName;
	_outputScaleFileName = aScalingParams._outputScaleFileName;
	_maxMarkerMovement = aScalingParams._maxMarkerMovement;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmScalingParams to their null values.
 */
void SimmScalingParams::setNull()
{
	setupProperties();
	setType("SimmScalingParams");
	setName("");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmScalingParams::setupProperties()
{
	_scalingOrderProp.setName("scaling_order");
	Array<string> sorder("");
	_scalingOrderProp.setComment("Valid options are 'measurements', 'manual', possibly in sequence.");
	_scalingOrderProp.setValue(sorder);
	_propertySet.append(&_scalingOrderProp);

	_measurementSetProp.setName("SimmMeasurementSet");
	_measurementSetProp.setComment("Measurements consist of a 'MarkerPairSet' and a 'BodyScaleSet'");
	_propertySet.append(&_measurementSetProp);

	_scaleSetProp.setName("ScaleSet");
	_scaleSetProp.setComment("Scale factors to be used for 'manual' scaling. Used only in 'manual' scaling.");
	_propertySet.append(&_scaleSetProp);

	_markerFileNameProp.setName("marker_trial");
	_markerFileNameProp.setComment("Name of .trc file containing the time history of marker positions. Used only in 'measurement' based scaling.");
	_propertySet.append(&_markerFileNameProp);

	const double defaultTimeRange[] = {-1.0, -1.0};
	_timeRangeProp.setName("time_range");
	_timeRangeProp.setComment("Range of time in the markers trial to use for measurement-based scaling.");
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

void SimmScalingParams::registerTypes()
{
	Object::RegisterType(SimmMeasurement());
	Object::RegisterType(Scale());
	SimmMeasurement::registerTypes();
}

SimmScalingParams& SimmScalingParams::operator=(const SimmScalingParams &aScalingParams)
{
	// BASE CLASS
	Object::operator=(aScalingParams);

	copyData(aScalingParams);

	return(*this);
}
//______________________________________________________________________________
/**
* Get final scale set to be applied to the model. This could come from measurements
* or be specified manually.
*
* @param Model to be scaled (used to retrieve body names)
* @returns ScaleSet to be used for scaling the model (one entry per segment)
*/

const ScaleSet& SimmScalingParams::getScaleSet(SimmModel& aModel, const char* subjectDirectory) 
{

	int i;
	Array<double> unity(1.0, 3);

	const SimmBodySet& bodySet = aModel.getBodies();

	// Make a scale set with a Scale for each body.
	// Initialize all factors to 1.0.
	//
	for (i = 0; i < bodySet.getSize(); i++)
	{
		Scale* bodyScale = new Scale();
		bodyScale->setSegmentName(bodySet.get(i)->getName());
		bodyScale->setScaleFactors(unity);
		bodyScale->setApply(true);
		_theScaleSet.append(bodyScale);
	}

	try
	{
		/* Keep track if any scaling ends up being applied or not due to mssing file, markers etc.
		 * If none is applied a STRONGER warning is warranted since it's likely a problem */
		bool anyScalingDone = false;
		/* Make adjustments to _theScaleSet, in the user-specified order. */
		for (i = 0; i < _scalingOrder.getSize(); i++)
		{
			/* For measurements, measure the distance between a pair of markers
			 * in the model, and in the static pose. The latter divided by the
			 * former is the scale factor. Put that scale factor in _theScaleSet,
			 * using the body/axis names specified in the measurement to
			 * determine in what place[s] to put the factor.
			 */
			if (_scalingOrder[i] == "measurements")
			{
				/* Load the static pose marker file, and average all the
				 * frames in the user-specified time range.
			    */

				SimmMarkerData staticPose(string(subjectDirectory)+_markerFileName);
				staticPose.averageFrames(_maxMarkerMovement, _timeRange[0], _timeRange[1]);
				staticPose.convertToUnits(aModel.getLengthUnits());

				/* Now take and apply the measurements. */
				for (int j = 0; j < _measurementSet.getSize(); j++)
				{
					if (_measurementSet[j]->getApply())
					{
						double scaleFactor = 1.0;
						double modelLength = aModel.takeMeasurement(*_measurementSet[j]);
						double staticPoseLength = staticPose.takeMeasurement(*_measurementSet[j]);
						if (modelLength != rdMath::NAN && staticPoseLength != rdMath::NAN)
						{
							scaleFactor = staticPoseLength / modelLength;
							_measurementSet[j]->applyScaleFactor(scaleFactor, _theScaleSet);
							anyScalingDone = true;
							cout << "Measurement " << _measurementSet[j]->getName() << ": model = " << modelLength << ", static pose = " << staticPoseLength << endl;
						}
						else
						{
							cout << "___WARNING___: " << _measurementSet[j]->getName() << " measurement not used to scale " << aModel.getName() << endl;
						}
					}
				}
			}
			/* For manual scales, just copy the XYZ scale factors from
		  	 * the manual scale into theScaleSet.
	  		 */
			else if (_scalingOrder[i] == "manual")
			{
				if (_scaleSet.getSize()==0){
					string message= "___WARNING___: Scale set is empty even though manual scale was selected.";
					throw( Exception(message,__FILE__,__LINE__) );
				}
				for (int j = 0; j < _scaleSet.getSize(); j++)
				{
					if (_scaleSet[j]->getApply())
					{
						const string& bodyName = _scaleSet[j]->getSegmentName();
						Array<double> factors(1.0, 3);
						_scaleSet[j]->getScaleFactors(factors);
						for (int k = 0; k < _theScaleSet.getSize(); k++)
						{
							if (_theScaleSet[k]->getSegmentName() == bodyName)
								_theScaleSet[k]->setScaleFactors(factors);
						}
						anyScalingDone = true;
					}
				}
			}
			else
			{
				// TODO: print error
			}
		}

		if (!anyScalingDone){
			cout << "___WARNING___: NO SCALING HAS BEEN APPLIED TO " << aModel.getName() << endl;
		}
	}
	catch (Exception &x)
	{
		x.print(cout);
		cout << "Press Return to continue. " << endl;
		cout.flush();
		int c = getc( stdin );
	}
	return _theScaleSet;

}
/**
 * Post scaling, write output files specified in the SimmScalingParams block.
 *
 * @params aModel: scaled model
 */
void SimmScalingParams::writeOutputFiles(SimmModel *aModel, const char* aPath)
{
	string path=(aPath==0)?"":string(aPath);

	/* Write output files, if names specified by the user. */
	if (!_outputJointFileNameProp.getUseDefault())
		aModel->writeSIMMJointFile(path+_outputJointFileName);

	if (!_outputMuscleFileNameProp.getUseDefault())
		aModel->writeSIMMMuscleFile(path+_outputMuscleFileName);

	if (!_outputModelFileNameProp.getUseDefault())
	{
		if (aModel->print(path+_outputModelFileName))
			cout << "Wrote model file " << _outputModelFileName << " from model " << aModel->getName() << endl;
	}

	if (!_outputScaleFileNameProp.getUseDefault())
	{
		if (_theScaleSet.print(path+_outputScaleFileName))
			cout << "Wrote scale file " << _outputScaleFileName << " for model " << aModel->getName() << endl;
	}
}

void SimmScalingParams::peteTest() const
{
	int i;
	Array<double> factors(0.0);

	cout << "   ScalingParams: " << getName() << endl;
	cout << "      scalingOrder: " << _scalingOrder << endl;
	for (i = 0; i < _measurementSet.getSize(); i++)
		_measurementSet[i]->peteTest();
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
