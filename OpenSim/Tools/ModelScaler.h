#ifndef __ModelScaler_h__
#define __ModelScaler_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  ModelScaler.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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


// INCLUDE
#include <OpenSim/Common/ScaleSet.h>
#include "MeasurementSet.h"

namespace SimTK {
class State;
}

namespace OpenSim {

class MarkerData;
class Model;

//=============================================================================
//=============================================================================
/**
 * A class for scaling a model. The default method of scaling involves
 * measuring distances between pairs of markers on the model and in a
 * static pose to determine scale factors.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMTOOLS_API ModelScaler : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(ModelScaler, Object);

//=============================================================================
// DATA
//=============================================================================
private:

protected:
    // whether or not to apply scaling
    PropertyBool _applyProp;
    bool &_apply;

    // order of the two scaling components: measurements and manual scaling
    PropertyStrArray _scalingOrderProp;
    Array<std::string>& _scalingOrder;

    // set of measurements to make on generic model and subject's static pose
    PropertyObj _measurementSetProp;
    MeasurementSet &_measurementSet;

    // set of XYZ scale factors to use for manual scaling
    PropertyObj _scaleSetProp;
    ScaleSet &_scaleSet;

    // name of marker file that contains the static pose
    PropertyStr _markerFileNameProp;
    std::string &_markerFileName;

    // range of frames to average in static pose file, specified by time
    PropertyDblArray _timeRangeProp;
    Array<double> &_timeRange;

    // whether or not to preserve mass distribution in generic model file when scaling
    PropertyBool _preserveMassDistProp;
    bool &_preserveMassDist;

    // name of XML model file to write when done scaling
    PropertyStr _outputModelFileNameProp;
    std::string &_outputModelFileName;

    // name of scale file to write when done scaling
    PropertyStr _outputScaleFileNameProp;
    std::string &_outputScaleFileName;

    // Whether or not to write to the designated output files (GUI will set this to false)
    bool _printResultFiles;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    ModelScaler();
    ModelScaler(const ModelScaler &aModelScaler);
    virtual ~ModelScaler();

#ifndef SWIG
    ModelScaler& operator=(const ModelScaler &aModelScaler);
#endif
   void copyData(const ModelScaler &aModelScaler);

    bool processModel(Model* aModel, const std::string& aPathToSubject="",
            double aFinalMass = -1.0) const;
    /* Register types to be used when reading a ModelScaler object from xml file. */
    static void registerTypes();

    /**
     * add a measurement
     */
    void addMeasurement(Measurement* aMeasurement)
    {
        _measurementSet.adoptAndAppend(aMeasurement);
    }
    /**
     * add a scale factor to current scaleSet
     */
    void addScale(Scale *aScale)
    {
        _scaleSet.adoptAndAppend(aScale);
    }
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------

    bool getApply() const { return _apply; }
    void setApply(bool aApply) { 
        _apply = aApply; 
        _applyProp.setValueIsDefault(false); 
    }

    MeasurementSet& getMeasurementSet() { return _measurementSet; }
    void setMeasurementSet(MeasurementSet& measurementSet) {
        _measurementSet = measurementSet;
    }

    ScaleSet& getScaleSet() { return _scaleSet; }
    void setScaleSetFile(const std::string& aScaleSetFilename) {
        _scaleSet = ScaleSet(aScaleSetFilename);
    }

    const Array<double> &getTimeRange() const { return _timeRange; }
    void setTimeRange(Array<double> timeRange) {
        _timeRange = timeRange;
        _timeRangeProp.setValueIsDefault(false);
    }

    bool getPreserveMassDist() const { return _preserveMassDist; }
    void setPreserveMassDist(bool preserveMassDist) {
        _preserveMassDist = preserveMassDist;
        _preserveMassDistProp.setValueIsDefault(false);
    }

    Array<std::string>& getScalingOrder() { return _scalingOrder; }
    void setScalingOrder(Array<std::string>& scalingOrder) {
        _scalingOrder = scalingOrder;
        _scalingOrderProp.setValueIsDefault(false);
    }

    const std::string& getMarkerFileName() const { return _markerFileName; }
    void setMarkerFileName(const std::string& aMarkerFileName) {
        _markerFileName = aMarkerFileName;
        _markerFileNameProp.setValueIsDefault(false);
    }

    const std::string& getOutputModelFileName() const { return _outputModelFileName; }
    void setOutputModelFileName(const std::string& aOutputModelFileName) {
        _outputModelFileName = aOutputModelFileName;
        _outputModelFileNameProp.setValueIsDefault(false);
    }

    const std::string& getOutputScaleFileName() const { return _outputScaleFileName; }
    void setOutputScaleFileName(const std::string& aOutputScaleFileName) {
        _outputScaleFileName = aOutputScaleFileName;
        _outputScaleFileNameProp.setValueIsDefault(false);
    }

    void setPrintResultFiles(bool aToWrite) { _printResultFiles = aToWrite; }

    double computeMeasurementScaleFactor(const SimTK::State& s, const Model& aModel, const MarkerData& aMarkerData, const Measurement& aMeasurement) const;
private:
    void setNull();
    void setupProperties();
    double takeModelMeasurement(const SimTK::State& s, const Model& aModel, const std::string& aName1, const std::string& aName2, const std::string& aMeasurementName) const;
    double takeExperimentalMarkerMeasurement(const MarkerData& aMarkerData, const std::string& aName1, const std::string& aName2, const std::string& aMeasurementName) const;

//=============================================================================
};  // END of class ModelScaler
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ModelScaler_h__


