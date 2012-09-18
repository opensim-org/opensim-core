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
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Michael A. Sherman                                              *
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
// none yet

namespace OpenSim {

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

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
class ModelDisplayHints {
public:
    /** Default construction creates a valid display hints object with all 
    hints set to their default values. **/
    ModelDisplayHints() {clear();}

    /** @name     Methods for specifying what should get displayed **/
    /**@{**/

    /** Specify whether wrap geometry should be displayed. 
    The default is yes. **/
    ModelDisplayHints& setShowWrapGeometry(bool showWrap) 
    {   _showWrapGeometry=showWrap; return *this; }
    /** Return current setting of the "show wrap geometry" flag. **/
    bool getShowWrapGeometry() const {return _showWrapGeometry;}

    /** Specify whether contact geometry should be displayed.
    The default is yes. **/
    ModelDisplayHints& setShowContactGeometry(bool showContact) 
    {   _showContactGeometry=showContact; return *this; }
    /** Return current setting of the "show contact geometry" flag. **/
    bool getShowContactGeometry() const {return _showContactGeometry;}

    /** Specify whether muscle path lines (or path lines for other path-using
    components like ligaments) should be displayed. The default is yes. **/
    ModelDisplayHints& setShowMusclePaths(bool showMusclePaths) 
    {   _showMusclePaths=showMusclePaths; return *this; }
    /** Return current setting of the "show muscle paths" flag. **/
    bool getShowMusclePaths() const {return _showMusclePaths;}

    /** Specify whether path points should appear along muscle paths in the 
    display. The default is yes but has no effect if muscle paths
    are not being displayed. **/
    ModelDisplayHints& setShowPathPoints(bool showPathPoints) 
    {   _showPathPoints=showPathPoints; return *this; }
    /** Return current setting of the "show path points" flag. **/
    bool getShowPathPoints() const {return _showPathPoints;}

    /** Specify whether marker points should be displayed.
    The default is yes. **/
    ModelDisplayHints& setShowMarkers(bool showMarkers) 
    {   _showMarkers=showMarkers; return *this; }
    /** Return current setting of the "show markers" flag. **/
    bool getShowMarkers() const {return _showMarkers;}

    /** Specify whether forces should be generated and displayed. This may 
    apply to forces applied by a component, or calculated reaction forces. The 
    default is no. **/
    ModelDisplayHints& setShowForces(bool showForces) 
    {   _showForces=showForces; return *this; }
    /** Return current setting of the "show forces" flag. **/
    bool getShowForces() const {return _showForces;}

    /** Specify whether coordinate frames should be displayed. The 
    default is no. **/
    ModelDisplayHints& setShowFrames(bool showFrames) 
    {   _showFrames=showFrames; return *this; }
    /** Return current setting of the "show frames" flag. **/
    bool getShowFrames() const {return _showFrames;}

    /** Specify whether text labels should be generated and displayed. The 
    default is no. **/
    ModelDisplayHints& setShowLabels(bool showLabels) 
    {   _showLabels=showLabels; return *this; }
    /** Return current setting of the "show labels" flag. **/
    bool getShowLabels() const {return _showLabels;}

    /** Specify whether debug geometry should be generated and displayed. The 
    default is no. **/
    ModelDisplayHints& setShowDebugGeometry(bool showDebugGeometry) 
    {   _showDebugGeometry=showDebugGeometry; return *this; }
    /** Return current setting of the "show debug geometry" flag. **/
    bool getShowDebugGeometry() const {return _showDebugGeometry;}
    /**@}**/

    /** This method sets all the display hints to their default values. **/
    void clear() {
        _showWrapGeometry = _showContactGeometry = _showMusclePaths =
        _showPathPoints = _showMarkers = true;
        _showForces = _showFrames = _showLabels = _showDebugGeometry = false;
    }
private:
    // Fields should match the clear() method; use order and alignment so that
    // any omissions can be spotted immediately.
    bool _showWrapGeometry, _showContactGeometry, _showMusclePaths, 
         _showPathPoints, _showMarkers;                     // default=yes
    bool _showForces, _showFrames, _showLabels, _showDebugGeometry; //=no
    // Add new fields to the end of the class to allow some hope for 
    // backwards compatibility.
};


//==============================================================================
} // end of namespace OpenSim

#endif // OPENSIM_MODEL_DISPLAY_HINTS_H_

