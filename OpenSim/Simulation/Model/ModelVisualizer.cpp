/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ModelVisualizer.cpp                        *
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
#include <OpenSim/Simulation/Wrap/WrapCylinder.h>
#include <OpenSim/Simulation/Wrap/WrapEllipsoid.h>
#include <OpenSim/Simulation/Wrap/WrapSphere.h>

#include <OpenSim/Simulation/Model/ContactGeometrySet.h>
#include <OpenSim/Simulation/Model/ContactSphere.h>
#include <OpenSim/version.h>

#include "Model.h"
#include "ModelDisplayHints.h"
#include "ModelVisualizer.h"
#include "MarkerSet.h"
#include "BodySet.h"
#include "ForceSet.h"

#include "Simbody.h"

#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::clog;
using std::endl;
#include <fstream>

using namespace OpenSim;
using namespace SimTK;

//==============================================================================
//                       OPENSIM INPUT LISTENER
//==============================================================================

// These constants are used to identify the OpenSim display menu in the
// Visualizer window and particular selections from it.
static const int ShowMenuId = 1;
static const int ToggleWrapGeometry = 0;
static const int ToggleContactGeometry = 1;
static const int ToggleMusclePaths = 2;
static const int TogglePathPoints = 3;
static const int ToggleMarkers = 4;
static const int ToggleFrames = 5;
static const int ToggleDefaultGeometry = 6;

/* This class gets first crack at user input coming in through the Visualizer
window. We use it to intercept anything we want to handle as part of the
standard OpenSim-provided interface, such as turning on and off display of wrap
objects. Anything we don't handle here will just get passed on to the
Visualizer's InputSilo where it will remain until the user's program goes
looking for it. */
class OpenSimInputListener : public Visualizer::InputListener {
public:
    OpenSimInputListener(Model& model) : _model(model) {}

    /* This is the implementation of the InputListener interface. We're going
    to override only the menu-pick method and ignore anything we don't
    recognize. Caution: this is being called from the Visualizer's input thread,
    *not* the main execution thread. Synchronization is required to do anything
    complicated; here we're just setting/clearing visualization flags so no
    synchronization is required. Note that the Visualizer's InputSilo class
    handles synchronization automatically but this one does not. */
    /*virtual*/ bool menuSelected(int menu, int item) {
        if (menu != ShowMenuId) return false; // some other menu
        ModelDisplayHints& hints = _model.updDisplayHints();
        switch(item) {
        case ToggleWrapGeometry:
            hints.setShowWrapGeometry(!hints.getShowWrapGeometry());
            return true; // absorb this input
        case ToggleContactGeometry:
            hints.setShowContactGeometry(!hints.getShowContactGeometry());
            return true;
        case ToggleMusclePaths:
            hints.setShowMusclePaths(!hints.getShowMusclePaths());
            return true;
        case TogglePathPoints:
            hints.setShowPathPoints(!hints.getShowPathPoints());
            return true;
        case ToggleMarkers:
            hints.setShowMarkers(!hints.getShowMarkers());
            return true;
        case ToggleDefaultGeometry: {
            SimbodyMatterSubsystem& matter =
                _model.updMatterSubsystem();
            matter.setShowDefaultGeometry(!matter.getShowDefaultGeometry());
            return true;
        }
        };
        return false; // let someone else deal with this input
    }
private:
    Model&  _model;
};





// Draw a path point with a small body-axis-aligned cross centered on
// the point.
void DefaultGeometry::drawPathPoint(const MobilizedBodyIndex&             body,
                                    const Vec3&                           pt_B,
                                    const Vec3&                           color,
                                    Array_<SimTK::DecorativeGeometry>&    geometry)
{
    geometry.push_back(DecorativeSphere(0.005)
                       .setTransform(pt_B)
                       .setBodyId(body)
                       .setColor(color)
                       .setOpacity(0.8));
}

void DefaultGeometry::generateDecorations
(const State&                         state,
 Array_<SimTK::DecorativeGeometry>&   geometry)
{
    const SimbodyMatterSubsystem& matter = _model.getMatterSubsystem();
    const ModelDisplayHints&      hints  = _model.getDisplayHints();

    // Display markers.
    if (hints.getShowMarkers()) {
        const Vec3 pink(1,.6,.8);
        const MarkerSet& markers = _model.getMarkerSet();
        for (int i=0; i < markers.getSize(); ++i) {
            const Marker& marker = markers[i];
            const OpenSim::Body& body = marker.getBody();
            const Vec3& p_BM = marker.getOffset();
            geometry.push_back(
                DecorativeSphere(_dispMarkerRadius).setBodyId(body.getIndex())
                .setColor(pink).setOpacity(_dispMarkerOpacity)
                .setTransform(marker.getOffset()));
        }
    }


    // Display wrap objects.
    if (hints.getShowWrapGeometry()) {
        const Vec3 color(SimTK::Cyan);
        Transform ztoy;
        ztoy.updR().setRotationFromAngleAboutX(SimTK_PI/2);
        const BodySet& bodies = _model.getBodySet();
        for (int i = 0; i < bodies.getSize(); i++) {
            const OpenSim::Body& body = bodies[i];
            const Transform& X_GB =
                matter.getMobilizedBody(body.getIndex()).getBodyTransform(state);
            const WrapObjectSet& wrapObjects = body.getWrapObjectSet();
            for (int j = 0; j < wrapObjects.getSize(); j++) {
                const string type = wrapObjects[j].getConcreteClassName();
                if (type == "WrapCylinder") {
                    const WrapCylinder* cylinder =
                        dynamic_cast<const WrapCylinder*>(&wrapObjects[j]);
                    if (cylinder != NULL) {
                        Transform X_GW = X_GB*cylinder->getTransform()*ztoy;
                        geometry.push_back(
                            DecorativeCylinder(cylinder->getRadius(),
                                               cylinder->getLength()/2)
                            .setTransform(X_GW).setResolution(_dispWrapResolution)
                            .setColor(color).setOpacity(_dispWrapOpacity));
                    }
                }
                else if (type == "WrapEllipsoid") {
                    const WrapEllipsoid* ellipsoid =
                        dynamic_cast<const WrapEllipsoid*>(&wrapObjects[j]);
                    if (ellipsoid != NULL) {
                        Transform X_GW = X_GB*ellipsoid->getTransform();
                        geometry.push_back(
                            DecorativeEllipsoid(ellipsoid->getRadii())
                            .setTransform(X_GW).setResolution(_dispWrapResolution)
                            .setColor(color).setOpacity(_dispWrapOpacity));
                    }
                }
                else if (type == "WrapSphere") {
                    const WrapSphere* sphere =
                        dynamic_cast<const WrapSphere*>(&wrapObjects[j]);
                    if (sphere != NULL) {
                        Transform X_GW = X_GB*sphere->getTransform();
                        geometry.push_back(
                            DecorativeSphere(sphere->getRadius())
                            .setTransform(X_GW).setResolution(_dispWrapResolution)
                            .setColor(color).setOpacity(_dispWrapOpacity));
                    }
                }
            }
        }
    }


    // Display contact geometry objects.
    if (hints.getShowContactGeometry()) {
        const Vec3 color(SimTK::Green);
        Transform ztoy;
        ztoy.updR().setRotationFromAngleAboutX(SimTK_PI/2);
        const ContactGeometrySet& contactGeometries = _model.getContactGeometrySet();

        for (int i = 0; i < contactGeometries.getSize(); i++) {
            const OpenSim::Body& body = contactGeometries.get(i).getBody();
            const Transform& X_GB =
                matter.getMobilizedBody(body.getIndex()).getBodyTransform(state);
            const string type = contactGeometries.get(i).getConcreteClassName();
            const int displayPref = contactGeometries.get(i).getDisplayPreference();
            //cout << type << ": " << contactGeometries.get(i).getName() << ": disp pref = " << displayPref << endl;

            if (type == "ContactSphere" && displayPref == 4) {
                ContactSphere* sphere =
                    dynamic_cast<ContactSphere*>(&contactGeometries.get(i));
                if (sphere != NULL) {
                    Transform X_GW = X_GB*sphere->getTransform();
                    geometry.push_back(
                        DecorativeSphere(sphere->getRadius())
                        .setTransform(X_GW).setResolution(_dispContactResolution)
                        .setColor(color).setOpacity(_dispContactOpacity));
                }
            }
        }
    }


    // Ask all the ModelComponents to generate dynamic geometry.
    _model.generateDecorations(false, _model.getDisplayHints(),
                               state, geometry);
}

//==============================================================================
//                            MODEL VISUALIZER
//==============================================================================

void ModelVisualizer::show(const SimTK::State& state) const {
    // Make sure we're realized at least through Velocity stage.
    _model.getMultibodySystem().realize(state, SimTK::Stage::Velocity);
    getSimbodyVisualizer().report(state);
}

// See if we can find the given file. The rules are
//  - if it is an absolute pathname, we only get one shot, else:
//  - define "modelDir" to be the absolute pathname of the
//      directory from which we read in the .osim model, if we did,
//      otherwise modelDir="." (current directory).
//  - look for the geometry file in modelDir
//  - look for the geometry file in modelDir/Geometry
//  - look for the geometry file in installDir/Geometry
bool ModelVisualizer::
findGeometryFile(const std::string&          geoFile,
                 bool&                       geoFileIsAbsolute,
                 SimTK::Array_<std::string>& attempts) const
{
    attempts.clear();
    std::string geoDirectory, geoFileName, geoExtension;
    SimTK::Pathname::deconstructPathname(geoFile,
                                         geoFileIsAbsolute, geoDirectory, geoFileName, geoExtension);

    bool foundIt = false;
    if (geoFileIsAbsolute) {
        attempts.push_back(geoFile);
        foundIt = Pathname::fileExists(attempts.back());
    } else {
        const string geoDir = "Geometry" + Pathname::getPathSeparator();
        string modelDir;
        if (_model.getInputFileName() == "Unassigned")
            modelDir = Pathname::getCurrentWorkingDirectory();
        else {
            bool isAbsolutePath;
            string directory, fileName, extension;
            SimTK::Pathname::deconstructPathname(
                _model.getInputFileName(),
                isAbsolutePath, directory, fileName, extension);
            modelDir = isAbsolutePath
                       ? directory
                       : Pathname::getCurrentWorkingDirectory() + directory;
        }

        attempts.push_back(modelDir + geoFile);
        foundIt = Pathname::fileExists(attempts.back());

        if (!foundIt) {
            attempts.push_back(modelDir + geoDir + geoFile);
            foundIt = Pathname::fileExists(attempts.back());
        }

        if (!foundIt) {
            const string installDir =
                Pathname::getInstallDir("OPENSIM_HOME", "OpenSim");
            attempts.push_back(installDir + geoDir + geoFile);
            foundIt = Pathname::fileExists(attempts.back());
        }
    }

    return foundIt;
}

// Call this on a newly-constructed ModelVisualizer (typically from the Model's
// initSystem() method) to set up the various auxiliary classes used for
// visualization and user interaction. This involves modifications to the
// System that must be done prior to realizeTopology(), and may modify the
// Model also.
void ModelVisualizer::createVisualizer() {
    _model.updMatterSubsystem().setShowDefaultGeometry(false);

    // Allocate a Simbody Visualizer. If environment variable
    // OPENSIM_HOME is set, add its bin subdirectory to the search path
    // for the SimbodyVisualizer executable. The search will go as
    // follows: first look in the same directory as the currently-
    // executing executable; then look in the $OPENSIM_HOME/bin
    // directory, then look in various default Simbody places.
    Array_<String> searchPath;
    if (SimTK::Pathname::environmentVariableExists("OPENSIM_HOME")) {
        searchPath.push_back(
            SimTK::Pathname::getEnvironmentVariable("OPENSIM_HOME")
            + "/bin");
    }
    _viz = new SimTK::Visualizer(_model.getMultibodySystem(),
                                 searchPath);

    // Make the Simbody Visualizer (that is, the display window) kill itself
    // when the API-side connection is lost (because the Visualizer object gets
    // destructed). Otherwise it will hang around afterwards.
    _viz->setShutdownWhenDestructed(true);

    _viz->setCameraClippingPlanes(.01,100.);
    _viz->setBackgroundColor(SimTK::Black);
    _viz->setBackgroundType(SimTK::Visualizer::SolidColor);

    // Give it an OpenSim-friendly window heading.
    bool isAbsolutePath;
    string directory, fileName, extension;
    SimTK::Pathname::deconstructPathname(
        SimTK::Pathname::getThisExecutablePath(),
        isAbsolutePath, directory, fileName, extension);
    _viz->setWindowTitle("OpenSim " + OpenSim::GetVersion()
                         + ": " + fileName + " (" + _model.getName() + ")");

    // Create a menu for choosing what to display.
    SimTK::Array_< std::pair<SimTK::String, int> > selections;
    selections.push_back(std::make_pair("Wrap geometry",
                                        ToggleWrapGeometry));
    selections.push_back(std::make_pair("Contact geometry",
                                        ToggleContactGeometry));
    selections.push_back(std::make_pair("Muscle paths",ToggleMusclePaths));
    selections.push_back(std::make_pair("Path points",TogglePathPoints));
    selections.push_back(std::make_pair("Markers",ToggleMarkers));
    selections.push_back(std::make_pair("Frames",ToggleFrames));
    selections.push_back(std::make_pair("Default geometry",
                                        ToggleDefaultGeometry));
    _viz->addMenu("Show", ShowMenuId, selections);

    // Add a DecorationGenerator to dispatch runtime generateDecorations()
    // calls.
    _decoGen = new DefaultGeometry(_model);
    _viz->addDecorationGenerator(_decoGen);

    // Add an input listener to handle display menu picks.
    _viz->addInputListener(new OpenSimInputListener(_model));

    // Allocate an InputSilo to pick up anything the above listener doesn't.
    _silo = new SimTK::Visualizer::InputSilo();
    _viz->addInputListener(_silo);

    // This is used for regular output of frames during forward dynamics.
    // TODO: allow user control of timing.
    _model.updMultibodySystem().addEventReporter
    (new SimTK::Visualizer::Reporter(*_viz, 1./30));
}

// We also rummage through the model to find fixed geometry that should be part
// of every frame. The supplied State must be realized through Instance stage.
void ModelVisualizer::collectFixedGeometry(const State& state) const {
    // Run through all the bodies and try to open the meshes associated
    // with them.
    const BodySet& bodies = _model.getBodySet();
    for (int i=0; i < bodies.getSize(); ++i) {
        const Body& body = bodies[i];
        const MobilizedBodyIndex bx = body.getIndex();
        const VisibleObject& visible = *body.getDisplayer();
        Vec3 scale;
        visible.getScaleFactors(scale);
        const Transform X_BV = visible.getTransform();
        const GeometrySet&   geomSet = visible.getGeometrySet();
        for (int g=0; g < geomSet.getSize(); ++g) {
            const DisplayGeometry& geo = geomSet[g];
            const DisplayGeometry::DisplayPreference pref = geo.getDisplayPreference();
            DecorativeGeometry::Representation rep;
            switch(pref) {
            case DisplayGeometry::None:
                continue; // don't bother with this one (TODO: is that right)
            case DisplayGeometry::WireFrame:
                rep=DecorativeGeometry::DrawWireframe;
                break;
            case DisplayGeometry::SolidFill:
            case DisplayGeometry::FlatShaded:
            case DisplayGeometry::GouraudShaded:
                rep = DecorativeGeometry::DrawSurface;
                break;
            default:
                assert(!"bad DisplayPreference");
            };

            const std::string& file = geo.getGeometryFile();
            bool isAbsolutePath;
            string directory, fileName, extension;
            SimTK::Pathname::deconstructPathname(file,
                                                 isAbsolutePath, directory, fileName, extension);
            const string lowerExtension = SimTK::String::toLower(extension);
            if (lowerExtension != ".vtp" && lowerExtension != ".obj") {
                std::clog << "ModelVisualizer ignoring '" << file
                          << "'; only .vtp and .obj files currently supported.\n";
                continue;
            }

            // File is a .vtp or .obj. See if we can find it.
            Array_<string> attempts;
            bool foundIt = findGeometryFile(file, isAbsolutePath, attempts);

            if (!foundIt) {
                std::clog << "ModelVisualizer couldn't find file '" << file
                          << "'; tried\n";
                for (unsigned i=0; i < attempts.size(); ++i)
                    std::clog << "  " << attempts[i] << "\n";
                if (!isAbsolutePath &&
                        !Pathname::environmentVariableExists("OPENSIM_HOME"))
                    std::clog << "Set environment variable OPENSIM_HOME "
                              << "to search $OPENSIM_HOME/Geometry.\n";
                continue;
            }

            SimTK::PolygonalMesh pmesh;
            try {
                if (lowerExtension == ".vtp") {
                    pmesh.loadVtpFile(attempts.back());
                } else {
                    std::ifstream objFile;
                    objFile.open(attempts.back().c_str());
                    pmesh.loadObjFile(objFile);
                    // objFile closes when destructed
                }
            } catch(const std::exception& e) {
                std::clog << "ModelVisualizer couldn't read "
                          << attempts.back() << " because:\n"
                          << e.what() << "\n";
                continue;
            }

            DecorativeMesh dmesh(pmesh);
            dmesh.setColor(geo.getColor());
            dmesh.setOpacity(geo.getOpacity());
            const Vec3 netScale = geo.getScaleFactors()
                                  .elementwiseMultiply(scale);
            dmesh.setScaleFactors(netScale);
            _viz->addDecoration(bx, X_BV*geo.getTransform(), dmesh);
        }
    }

    // Collect any fixed geometry from the ModelComponents.
    Array_<DecorativeGeometry> fixedGeometry;
    _model.generateDecorations
    (true, _model.getDisplayHints(), state, fixedGeometry);

    for (unsigned i=0; i < fixedGeometry.size(); ++i) {
        const DecorativeGeometry& dgeo = fixedGeometry[i];
        _viz->addDecoration(MobilizedBodyIndex(dgeo.getBodyId()),
                            Transform(), dgeo);
    }
}
