#ifndef OPENSIM_INVERSE_KINEMATICS_TOOL_H_
#define OPENSIM_INVERSE_KINEMATICS_TOOL_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  InverseKinematicsTool.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Tools/IKTaskSet.h>
#include <OpenSim/Tools/InverseKinematicsToolBase.h>

namespace OpenSim {

class Model;
class MarkersReference;
class CoordinateReference;

//=============================================================================
//=============================================================================
/**
 * A Tool that performs an Inverse Kinematics analysis with a given model.
 * Inverse kinematics is the solution of internal coordinates that poses
 * the model such that the landmark locations (markers), affixed to the model,
 * minimize the weighted least-squares error with observations of markers 
 * in spatial coordinates. Observations of coordinates can also be included.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMTOOLS_API InverseKinematicsTool : public InverseKinematicsToolBase {
    OpenSim_DECLARE_CONCRETE_OBJECT(InverseKinematicsTool, InverseKinematicsToolBase);

public:

    OpenSim_DECLARE_UNNAMED_PROPERTY(
            IKTaskSet, 
            "Markers and coordinates to be considered (tasks) and their weightings. "
            "The sum of weighted-squared task errors composes the cost function.");

    OpenSim_DECLARE_PROPERTY(marker_file, std::string,
            "TRC file (.trc) containing the time history of observations of marker "
            "positions obtained during a motion capture experiment. Markers in this "
            "file that have a corresponding task and model marker are included.");


    OpenSim_DECLARE_PROPERTY(coordinate_file, std::string,
            "The name of the storage (.sto or .mot) file containing the time "
            "history of coordinate observations. Coordinate values from this file are "
            "included if there is a corresponding model coordinate and task. ");

    OpenSim_DECLARE_PROPERTY(report_marker_locations, bool,
            "Flag indicating whether or not to report model marker locations. "
            "Note, model marker locations are expressed in Ground.");

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    virtual ~InverseKinematicsTool();
    InverseKinematicsTool();
    InverseKinematicsTool(const std::string &aFileName, bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;
 
    /* Handle reading older formats/Versioning */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1) override;

    //---- Setters and getters for various attributes
    void setMarkerDataFileName(const std::string& markerDataFileName) {
        upd_marker_file() = markerDataFileName;
    };
    const std::string& getMarkerDataFileName() const {
        return get_marker_file();
    };

    void setCoordinateFileName(const std::string& coordDataFileName) {
        upd_coordinate_file() = coordDataFileName;
    };
    const std::string& getCoordinateFileName() const {
        return get_coordinate_file();
    };

    IKTaskSet& getIKTaskSet() { return upd_IKTaskSet(); }

    //--------------------------------------------------------------------------
    // INTERFACE
    //--------------------------------------------------------------------------
    bool run() override SWIG_DECLARE_EXCEPTION;
#ifndef SWIG
    /** @cond **/ // hide from Doxygen
#endif
    // For testing/debugging it is necessary to know exactly what are the
    // MarkersReference (set of marker trajectories and their weights) and
    // CoordinateReferences that are being used by the InverseKinematicsSolver.
    void populateReferences(MarkersReference& markersReference,
        SimTK::Array_<CoordinateReference>&coordinateReferences) const;
#ifndef SWIG
    /** @endcond **/
#endif
private:
    void constructProperties();

    //=============================================================================
};  // END of class InverseKinematicsTool
//=============================================================================
} // namespace

#endif // OPENSIM_INVERSE_KINEMATICS_TOOL_H_
