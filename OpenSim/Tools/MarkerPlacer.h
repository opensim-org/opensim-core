#ifndef __MarkerPlacer_h__
#define __MarkerPlacer_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  MarkerPlacer.h                          *
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
#include "OpenSim/Tools/IKTaskSet.h"
#include "osimToolsDLL.h"

#include <SimTKcommon/internal/ResetOnCopy.h>

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Property.h>

namespace SimTK {
class State;
}

namespace OpenSim {

class Model;
class MarkerData;
class IKTrial;
class Storage;

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing how to place markers
 * on a model (presumably after it has been scaled to fit a subject).
 *
 * MarkerPlacer is bundled with ModelScaler and GenericModelMaker to 
 * form the ScaleTool
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMTOOLS_API MarkerPlacer : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MarkerPlacer, Object);

//=============================================================================
// DATA
//=============================================================================
public:
OpenSim_DECLARE_PROPERTY(
        apply, bool, "Whether or not to use the marker placer during scale");
OpenSim_DECLARE_PROPERTY(marker_file, std::string,
        "TRC file (.trc) containing the time history of experimental marker "
        "positions ");
OpenSim_DECLARE_LIST_PROPERTY_SIZE(time_range, double, 2,
        "Time range over which the marker positions are averaged.");
OpenSim_DECLARE_PROPERTY(IKTaskSet, IKTaskSet,
        "Task set used to specify weights used in the IK computation of the "
        "static pose.");
OpenSim_DECLARE_PROPERTY(coordinate_file, std::string,
        "Name of file containing the joint angles "
        "used to set the initial configuration of the model for the purpose of "
        "placing the markers. "
        "These coordinate values can also be included in the optimization "
        "problem used to place the markers. "
        "Before the model markers are placed, a single frame of an inverse "
        "kinematics (IK) problem is solved. "
        "The IK problem can be solved simply by matching marker positions, but "
        "if the model markers are not "
        "in the correct locations, the IK solution will not be very good and "
        "neither will marker placement. "
        "Alternatively, coordinate values (specified in this file) can be "
        "specified and used to influence the IK solution. "
        "This is valuable particularly if you have high confidence in the "
        "coordinate values. "
        "For example, you know for the static trial the subject was standing "
        "will all joint angles close to zero. "
        "If the coordinate set (see the CoordinateSet property) contains "
        "non-zero weights for coordinates, "
        "the IK solution will try to match not only the marker positions, but "
        "also the coordinates in this file. "
        "Least-squared error is used to solve the IK problem. ");
OpenSim_DECLARE_PROPERTY(output_model_file, std::string,
        "Output OpenSim model file (.osim) after scaling and maker placement.");
OpenSim_DECLARE_PROPERTY(output_marker_file, std::string,
        "Output marker set containing the new marker locations after markers "
        "have been placed.");
OpenSim_DECLARE_PROPERTY(output_motion_file, std::string,
        "Name of the motion file (.mot) written after marker relocation "
        "(optional).");
OpenSim_DECLARE_PROPERTY(max_marker_movement, double,
        "Maximum amount of movement allowed in marker data when averaging "
        "frames of the static trial. ");
OpenSim_DECLARE_PROPERTY(print_result_files, bool,
        "Whether or not to write to the designated output files (GUI will set "
        "this to false)");
OpenSim_DECLARE_PROPERTY(move_model_markers, bool,
        "Whether to move the model markers (set to false if you just want to "
        "preview the static pose)");

protected:
    // This is cached during processModel() so the GUI can access it.
    mutable SimTK::ResetOnCopy<std::unique_ptr<Storage>> _outputStorage;
    //=============================================================================
    // METHODS
    //=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    MarkerPlacer();
    virtual ~MarkerPlacer();

    bool processModel(
            Model* aModel, const std::string& aPathToSubject = "") const;

    /* Register types to be used when reading object from xml file. */
    static void registerTypes();
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------

    bool getApply() const { return get_apply(); }
    void setApply(bool aApply) { set_apply(aApply); }

    const std::string& getStaticPoseFileName() const {
        return get_marker_file();
    }
    void setStaticPoseFileName(const std::string &aFileName) 
    {
        set_marker_file(aFileName);
    }

    const Array<double> getTimeRange() const {
        return Array<double>{get_time_range(0), get_time_range(1)};
    }
    void setTimeRange(const Array<double>& timeRange) {
        set_time_range(timeRange);
    }
    const IKTaskSet& getIKTaskSet() const { return get_IKTaskSet(); }

    const std::string& getCoordinateFileName() const {
        return get_coordinate_file();
    }
    void setCoordinateFileName(const std::string& aCoordinateFileName)
    {
        set_coordinate_file(aCoordinateFileName);
    }

    const std::string& getMarkerFileName() const { return get_marker_file(); }
    void setMarkerFileName( const std::string& aMarkerFileName)
    {
        set_marker_file(aMarkerFileName);
    }

    const double& getMaxMarkerMovement() const {
        return get_max_marker_movement();
    }
    void setMaxMarkerMovement(double aMaxMarkerMovement)
    {
        set_max_marker_movement(aMaxMarkerMovement);
    }

    const std::string& getOutputModelFileName() const {
        return get_output_model_file();
    }
    void setOutputModelFileName(const std::string& aOutputModelFileName)
    {
        set_output_model_file(aOutputModelFileName);
    }

    const std::string& getOutputMarkerFileName() const {
        return get_output_marker_file();
    }
    void setOutputMarkerFileName(const std::string& outputMarkerFileName)
    {
        set_output_marker_file(outputMarkerFileName);
    }

    const std::string& getOutputMotionFileName() const {
        return get_output_motion_file();
    }
    void setOutputMotionFileName(const std::string& outputMotionFileName)
    {
        set_output_motion_file(outputMotionFileName);
    }

    void setPrintResultFiles(bool aToWrite) {
        set_print_result_files(aToWrite);
    }

    const bool& getMoveModelMarkers() { return get_move_model_markers(); }
    void setMoveModelMarkers(bool aMove) { set_move_model_markers(aMove); }

    Storage *getOutputStorage();


private:
    void constructProperties();
    void moveModelMarkersToPose(SimTK::State& s, Model& aModel,
            MarkerData& aPose) const;
//=============================================================================
};  // END of class MarkerPlacer
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MarkerPlacer_h__


