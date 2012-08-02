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
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Michael A. Sherman                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/** @file
This file provides an OpenSim-oriented interface to the Simbody Visualizer
that provides some visualization and user interaction when running a program
that uses the OpenSim API. **/

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "Simbody.h"

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

    /** Return a const reference to the Model for which this %ModelVisualizer
    was constructed. **/
    const Model& getModel() const {return _model;}
   
    /** Return a writable reference to the Model for which this %ModelVisualizer
    was constructed. **/
    Model& updModel() {return _model;}

    /** Given the name of a geometry file, this method will attempt to
    find it in a series of locations using the same algorithm as is done
    internally by the %ModelVisualizer. 
    
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
        absolute path name of \a geoFile.
    @returns \c true if \a geoFile was located and is readable.
        
    The search rule is as follows:
      - If \a geoFile is an absolute pathname no search is done.
      - Otherwise, define modelDir as the directory from which the current
        Model file was read in, if any, otherwise the current directory.
      - Try modelDir/geoFile, then modelDir/Geometry/geoFile.
      - Finally, try installDir/geoFile where installDir is taken from
        the OPENSIM_HOME environment variable if it exists, otherwise
        a default installation directory. 
    
    No attempt is made to validate the contents of the file or whether it
    has a supported extension; we're just looking for a file of the given
    name that exists and is readable. **/
    bool findGeometryFile(const std::string&          geoFile,
                          bool&                       isAbsolute,
                          SimTK::Array_<std::string>& attempts) const;
    /**@}**/
private:
    friend class Model;

    // Only Model is permitted to create or destruct one of these. Note that
    // this will cause modifications to System that must occur prior to 
    // realizeTopology().
    ModelVisualizer(Model& model) : _model(model), _viz(0) {
        clear();
        createVisualizer();
    }

    // Called from Model's initSystem() method; state must be realized 
    // through Instance stage.
    void collectFixedGeometry(const SimTK::State& state) const;

    ~ModelVisualizer() {clear();}

    void clear() {
        delete _viz; _viz = 0;
        _silo = 0; // Visualizer will have deleted this.
    }

    void createVisualizer();

private:
    Model&                  _model;
    SimTK::Visualizer*      _viz;

    // This is just a reference -- it is owned by the Simbody Visualizer so 
    // don't delete it!
    SimTK::Visualizer::InputSilo*   _silo;
};

} // namespace OpenSim

#endif // OPENSIM_MODEL_VISUALIZER_H_

