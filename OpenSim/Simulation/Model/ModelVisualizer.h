#ifndef OPENSIM_MODEL_VISUALIZER_H_
#define OPENSIM_MODEL_VISUALIZER_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ModelVisualizer.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

/** @file
This file provides an OpenSim-oriented interface to the Simbody Visualizer
that provides some visualization and user interaction when running a program
that uses the OpenSim API. **/

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <simbody/internal/Visualizer.h>

namespace OpenSim {
class Model;
}

#ifndef SWIG
namespace SimTK {

//==============================================================================
//                           DEFAULT GEOMETRY
//==============================================================================
// This class implements a SimTK DecorationGenerator. We'll add one to the
// Visualizer so it can invoke the generateDecorations() dispatcher to pick up 
// per-frame geometry.
class DefaultGeometry : public DecorationGenerator {
public:
    DefaultGeometry(OpenSim::Model& model) : _model(model) {
        _dispMarkerRadius = 0.005;
        _dispMarkerOpacity = 1.0;
        _dispWrapOpacity = 0.5;
        _dispWrapResolution = 2.0;
        _dispContactOpacity = 0.75;
        _dispContactResolution = 2.0;
    }
    void generateDecorations(const SimTK::State& state, 
                             SimTK::Array_<SimTK::DecorativeGeometry>& geometry) override;
    double getDispMarkerRadius() {return _dispMarkerRadius;}
    void   setDispMarkerRadius(double a) {_dispMarkerRadius=a;}
    double getDispMarkerOpacity() {return _dispMarkerOpacity;}
    void   setDispMarkerOpacity(double a) {_dispMarkerOpacity=a;}

    double getDispWrapOpacity() {return _dispWrapOpacity;}
    void   setDispWrapOpacity(double a) {_dispWrapOpacity=a;}
    double getDispWrapResolution() {return _dispWrapResolution;}
    void   setDispWrapResolution(double a) {_dispWrapResolution=a;}

    double getDispContactOpacity() {return _dispContactOpacity;}
    void   setDispContactOpacity(double a) {_dispContactOpacity=a;}
    double getDispContactResolution() {return _dispContactResolution;}
    void   setDispContactResolution(double a) {_dispContactResolution=a;}

static void drawPathPoint(const SimTK::MobilizedBodyIndex&             body,
                          const SimTK::Vec3&                           pt_B,
                          const SimTK::Vec3&                           color,
                          SimTK::Array_<SimTK::DecorativeGeometry>&    geometry);

private:
    OpenSim::Model&  _model;

    // Displayer internal variables
    double _dispMarkerRadius;
    double _dispMarkerOpacity;
    double _dispWrapOpacity;
    double _dispWrapResolution;
    double  _dispContactOpacity;
    double _dispContactResolution;
};

}
#endif

namespace OpenSim {

/** This class manages runtime visualization of a Model that is being 
manipulated through the OpenSim API. You should not allocate one of these
yourself; instead, call the Model's setUseVisualizer() method and let the
Model allocate one for itself. You may find the defaults to be adequate, but
you can also get access to the %ModelVisualizer if you need it by calling
the Model's getVisualizer() method. 

The %ModelVisualizer consults the Model's ModelDisplayHints object for 
instructions on what to display.

@author Michael Sherman

@see ModelDisplayHints, Model **/
class OSIMSIMULATION_API ModelVisualizer {
public:

    ~ModelVisualizer() {clear();}

    /** @name                Drawing methods
    Currently there is just a single method for generating a frame. **/
    /**@{**/
    /** Evaluate the geometry needed to visualize the given \a state and
    use it to generate a new image in the Visualizer window. **/
    void show(const SimTK::State& state) const;
    /**@}**/

    /** @name       Access to SimTK::Visualizer features
    These methods provide access to lower-level SimTK::Visualizer objects
    that are used in the implementation of this ModelVisualizer. **/
    /**@{**/

    /** If you want to poll for user input, you'll need access to the
    SimTK::Visualizer::InputSilo maintained here. Writable access is required
    to remove user input from the queues. **/
    const SimTK::Visualizer::InputSilo& getInputSilo() const {return *_silo;}
    /** Get writable access to the InputSilo so you can remove user input
    from the queues. **/
    SimTK::Visualizer::InputSilo& updInputSilo() {return *_silo;}

    /** If you want access to the underlying Simbody SimTK::Visualizer, you
    can get a const reference here. **/
    const SimTK::Visualizer& getSimbodyVisualizer() const 
    {   assert(_viz); return *_viz; }
    /** If you want writable access to the underlying Simbody SimTK::Visualizer,
    you can get a non-const reference here, provided that you have non-const
    access to the %ModelVisualizer. **/
    SimTK::Visualizer& updSimbodyVisualizer() 
    {   assert(_viz); return *_viz; }
    /**@}**/

    /** @name               Miscellaneous utilities
    Most users will not need to use these methods. **/
    /**@{**/

    /** Return a pointer to the DefaultGeometry decoration generator used by 
    this %ModelVisualizer. **/
    SimTK::DefaultGeometry* getGeometryDecorationGenerator() {return _decoGen;}

    /** Return a const reference to the Model for which this %ModelVisualizer
    was constructed. **/
    const Model& getModel() const {return _model;}
   
    /** Return a writable reference to the Model for which this %ModelVisualizer
    was constructed. **/
    Model& updModel() {return _model;}

    /** Given the name of a geometry file, this method will attempt to
    find it in a series of locations using the same algorithm as is done
    internally by the %ModelVisualizer. 
    
    @param[in]      model
        Used to obtain the name of the file from which the model was loaded.
    @param[in]      geoFile 
        Name of file to look for; can be absolute or relative path name or just
        a file name and the extension must be supplied.
    @param[out]     isAbsolute
        This output parameter is set to true on return if the supplied 
        \a geoFile was an absolute path name; in that case no searching was
        done.
    @param[out]     attempts
        On return, this is a list of the absolute path names that were tried.
        If \a geoFile was found, attempts.back() (the last entry) is the
        absolute path name of \a geoFile. The last entry of this array will be
        the path that succeeded in finding the geometry file.
    @returns \c true if \a geoFile was located and is readable.
        
    The search rule is as follows:
      - If \a geoFile is an absolute pathname no search is done.
      - Otherwise, define modelDir as the directory from which the current
        Model file was read in, if any, otherwise the current directory.
      - Try modelDir/geoFile, then modelDir/Geometry/geoFile.
      - Otherwise, try the search paths added through 
        addDirToGeometrySearchPaths(). The paths are searched in 
        reverse-chronological order -- the latest path added is searched first.
      - Finally, try installDir/geoFile where installDir is taken from
        the OPENSIM_HOME environment variable if it exists, otherwise
        a default installation directory. 
    
    No attempt is made to validate the contents of the file or whether it
    has a supported extension; we're just looking for a file of the given
    name that exists and is readable. **/
    static bool findGeometryFile(const Model& model,
                            const std::string&          geoFile,
                            bool&                       isAbsolute,
                            SimTK::Array_<std::string>& attempts);

    /** Add a directory to the search path to be used by the function
    findGeometryFile. The added paths are searched in the 
    reverse-chronological order -- the latest path added is searched first. */
    static void addDirToGeometrySearchPaths(const std::string& dir);
    /**@}**/


private:
    friend class Model;

    // Only Model is permitted to create one of these. Note that
    // this will cause modifications to System that must occur prior to 
    // realizeTopology().
    ModelVisualizer(Model& model) : _model(model), _viz(0) {
        clear();
        createVisualizer();
    }

    // Called from Model's initSystem() method; state must be realized 
    // through Instance stage.
    void collectFixedGeometry(const SimTK::State& state) const;

    void clear() {
        delete _viz; _viz = 0;
        _silo = 0; // Visualizer will have deleted this.
    }

    void createVisualizer();

private:
    Model&                       _model;
    SimTK::Visualizer*           _viz;
    SimTK::DefaultGeometry*      _decoGen;

    // This is just a reference -- it is owned by the Simbody Visualizer so 
    // don't delete it!
    SimTK::Visualizer::InputSilo*   _silo;

    // List of directories to search.
    static SimTK::Array_<std::string> dirsToSearch;
};







} // namespace OpenSim

#endif // OPENSIM_MODEL_VISUALIZER_H_

