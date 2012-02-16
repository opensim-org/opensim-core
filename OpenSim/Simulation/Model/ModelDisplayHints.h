#ifndef OPENSIM_MODEL_DISPLAY_HINTS_H_
#define OPENSIM_MODEL_DISPLAY_HINTS_H_

// ModelDisplayHints.h
// Authors: Michael Sherman
/*
 * Copyright (c)  2012, Stanford University. All rights reserved. 
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

