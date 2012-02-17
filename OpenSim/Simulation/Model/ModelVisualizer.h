#ifndef OPENSIM_MODEL_VISUALIZER_H_
#define OPENSIM_MODEL_VISUALIZER_H_

// ModelVisualizer.h
// Authors: Michael Sherman
/*
 * Copyright (c) 2012, Stanford University. All rights reserved.
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
    internally by the %ModelVisualizer. The absolute path names that were
    tried are returned in \a attempts. If the file is found the method returns
    \c true and attempts.back() is the absolute path name. The search rule
    is as follows:
      - If \c geoFile is an absolute pathname no search is done.
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

