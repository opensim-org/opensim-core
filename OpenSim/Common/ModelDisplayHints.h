#ifndef OPENSIM_MODEL_DISPLAY_HINTS_H_
#define OPENSIM_MODEL_DISPLAY_HINTS_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ModelDisplayHints.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Michael A. Sherman, Ayman Habib                                 *
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

// INCLUDES
#include <OpenSim/Common/osimCommonDLL.h>
#include <OpenSim/Common/Object.h>


namespace OpenSim {

//==============================================================================
//                           MODEL DISPLAY HINTS
//==============================================================================
/** This class captures information indicating user or programmer preferences 
to guide automatic display geometry generation by a Model. Fields here may be
set programmatically or as a result of user choices made in the GUI or 
ModelVisualizer. Any display-generating code including the GUI, ModelVisualizer,
and any geometry-generating ModelComponent will be supplied with one of these 
so that it can modify its behavior if there are any fields it recognizes. 

If you are writing a ModelComponent that generates some of its own geometry,
and you would like people to consider it well-behaved, you should check whether
any of the flags here might reasonably be expected to affect the geometry that
your component produces. The currently-supported flags are:
  - show wrap geometry
  - show contact geometry
  - show muscle paths (should apply to other path objects too)
  - show path points
  - show markers
  - show stations
  - show forces
  - show frames
  - show labels
  - show debug geometry

This class is intended to provide some minimal user control over generated
geometry in a form that is easy for a ModelComponent author to deal with, since
the OpenSim user interface won't know anything about that component or its
geometry. Nothing prevents a user interface from providing much more 
sophisticated control of display features that it understands and the OpenSim
GUI does that.

If you would like to see some additional generally-useful flags or options 
supported here, please file a feature request or post to the OpenSim forum on 
SimTK.org.

@note The "set" methods here return a reference to the object so they can
be chained for convenience like assignment statements.

@see ModelComponent::generateDecorations(), ModelVisualizer 
@author Michael Sherman **/
class OSIMCOMMON_API ModelDisplayHints : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModelDisplayHints, Object);
public:
    OpenSim_DECLARE_PROPERTY(show_wrap_geometry, bool,
        "Flag to indicate whether or not to show wrap geometry, default to true.");

    OpenSim_DECLARE_PROPERTY(show_contact_geometry, bool,
        "Flag to indicate whether or not to show contact geometry, default to true.");

    OpenSim_DECLARE_PROPERTY(show_path_geometry, bool,
        "Flag to indicate whether or not to show path geometry for forces and actuators, default to true.");

    OpenSim_DECLARE_PROPERTY(show_path_points, bool,
        "Flag to indicate whether or not to show points along path are shown, default to true.");

    OpenSim_DECLARE_PROPERTY(show_markers, bool,
        "Flag to indicate whether or not to show markers, default to true.");

    OpenSim_DECLARE_PROPERTY(show_stations, bool,
        "Flag to indicate whether or not to show stations, default to false.");

    OpenSim_DECLARE_PROPERTY(marker_color, SimTK::Vec3,
        "Color is RGB, each components is in the range [0, 1], default to pink.");

    OpenSim_DECLARE_PROPERTY(show_forces, bool,
        "Flag to indicate whether or not to show forces, default to true.");

    OpenSim_DECLARE_PROPERTY(show_frames, bool,
        "Flag to indicate whether or not to show frames, default to false.");

    OpenSim_DECLARE_PROPERTY(show_labels, bool,
        "Flag to indicate whether or not to show labels, default to false.");

    OpenSim_DECLARE_PROPERTY(show_debug_geometry, bool,
        "Flag to indicate whether or not to show debug geometry, default to false.");

    /** Default construction creates a valid display hints object with all
    hints set to their default values. **/
    ModelDisplayHints() { constructProperties(); }
    /** Turn off visualization completely, only use for API/modeling.
    Meshes will not be loaded, path-wrapping intermediate points will
    not be computed, MomentArm computations are not affected however.
    Intentionally there's no reverse API to turn on visualization downstream.
    **/
    void disableVisualization() { _visualization = false; }
    bool isVisualizationEnabled() const { return _visualization; }
private:
    void constructProperties() {
        constructProperty_show_wrap_geometry(true);
        constructProperty_show_contact_geometry(true);
        constructProperty_show_path_geometry(true);
        constructProperty_show_path_points(true);
        constructProperty_show_markers(true);
        constructProperty_show_stations(false);
        constructProperty_marker_color(SimTK::Vec3(1, .6, .8));
        constructProperty_show_frames(false);
        constructProperty_show_labels(false);
        constructProperty_show_forces(true);
        constructProperty_show_debug_geometry(false);
    }
    bool _visualization{true};
};


//==============================================================================
} // end of namespace OpenSim

#endif // OPENSIM_MODEL_DISPLAY_HINTS_H_

