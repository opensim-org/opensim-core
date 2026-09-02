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
#include "osimToolsDLL.h"

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Property.h>
#include <OpenSim/Tools/MeasurementSet.h>

namespace SimTK {
class State;
}

namespace OpenSim {

class MarkerData;
class Model;
class Measurement;
class ScaleSet;
class Scale;

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
public:
OpenSim_DECLARE_PROPERTY(
        apply, bool, "Whether or not to use the model scaler during scale");
OpenSim_DECLARE_LIST_PROPERTY(scaling_order, std::string,
        "Specifies the scaling method and order. "
        "Valid options are 'measurements', 'manualScale', singly or both in "
        "any sequence.");
OpenSim_DECLARE_PROPERTY(MeasurementSet, MeasurementSet,
        "Specifies the measurements by which body segments are to be scaled.");
OpenSim_DECLARE_PROPERTY(
        ScaleSet, ScaleSet, "Scale factors to be used for manual scaling.");
OpenSim_DECLARE_PROPERTY(marker_file, std::string,
        "TRC file (.trc) containing the marker positions used for "
        "measurement-based scaling. "
        "This is usually a static trial, but doesn't need to be.  The "
        "marker-pair distances are computed for each "
        "time step in the TRC file and averaged across the time range.");
OpenSim_DECLARE_LIST_PROPERTY_SIZE(time_range, double, 2,
        "Time range over which to average marker-pair distances in the marker "
        "file (.trc) for ");
OpenSim_DECLARE_PROPERTY(preserve_mass_distribution, bool,
        "Flag (true or false) indicating whether or not to preserve relative "
        "mass between segments.");
OpenSim_DECLARE_PROPERTY(output_model_file, std::string,
        "Name of OpenSim model file (.osim) to write when done scaling.");
OpenSim_DECLARE_PROPERTY(output_scale_file, std::string,
        "Name of file to write containing the scale factors that were applied "
        "to the unscaled model (optional).");
OpenSim_DECLARE_PROPERTY(print_result_files, bool,
        "Whether or not to write to the designated output files (GUI will set "
        "this to false)");

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    ModelScaler();

    bool processModel(Model* aModel, const std::string& aPathToSubject = "",
            double aFinalMass = -1.0) const;
    /* Register types to be used when reading a ModelScaler object from xml file. */
    static void registerTypes();

    /**
     * add a measurement
     */
    void addMeasurement(Measurement* aMeasurement);
    /**
     * add a scale factor to current scaleSet
     */
    void addScale(Scale* aScale);
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------

    bool getApply() const { return get_apply(); }
    void setApply(bool aApply) { set_apply(aApply); }

    MeasurementSet& getMeasurementSet() { return upd_MeasurementSet(); };
    void setMeasurementSet(MeasurementSet& measurementSet);

    ScaleSet& getScaleSet() { return upd_ScaleSet(); };
    void setScaleSetFile(const std::string& aScaleSetFilename);

    Array<double> getTimeRange() const {
        return Array<double>{get_time_range(0), get_time_range(1)};
    }
    void setTimeRange(Array<double> timeRange) { set_time_range(timeRange); }

    bool getPreserveMassDist() const {
        return get_preserve_mass_distribution();
    }
    void setPreserveMassDist(bool preserveMassDist) {
        set_preserve_mass_distribution(preserveMassDist);
    }

    Array<std::string> getScalingOrder() const;
    void setScalingOrder(const Array<std::string>& scalingOrder) {
        set_scaling_order(scalingOrder);
    }

    const std::string& getMarkerFileName() const { return get_marker_file(); }
    void setMarkerFileName(const std::string& aMarkerFileName) {
        set_marker_file(aMarkerFileName);
    }

    const std::string& getOutputModelFileName() const {
        return get_output_model_file();
    }
    void setOutputModelFileName(const std::string& aOutputModelFileName) {
        set_output_model_file(aOutputModelFileName);
    }

    const std::string& getOutputScaleFileName() const {
        return get_output_scale_file();
    }
    void setOutputScaleFileName(const std::string& aOutputScaleFileName) {
        set_output_scale_file(aOutputScaleFileName);
    }

    void setPrintResultFiles(bool aToWrite) {
        set_print_result_files(aToWrite);
    }

    double computeMeasurementScaleFactor(const SimTK::State& s, const Model& aModel, const MarkerData& aMarkerData, const Measurement& aMeasurement) const;
private:
    void constructProperties();
    double takeModelMeasurement(const SimTK::State& s, const Model& aModel, const std::string& aName1, const std::string& aName2, const std::string& aMeasurementName) const;
    double takeExperimentalMarkerMeasurement(const MarkerData& aMarkerData, const std::string& aName1, const std::string& aName2, const std::string& aMeasurementName) const;

//=============================================================================
};  // END of class ModelScaler
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ModelScaler_h__
