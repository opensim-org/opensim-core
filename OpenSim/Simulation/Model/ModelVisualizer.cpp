// ModelVisualizer.cpp
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
#include <OpenSim/Simulation/Wrap/WrapCylinder.h>
#include <OpenSim/Simulation/Wrap/WrapEllipsoid.h>
#include <OpenSim/Simulation/Wrap/WrapSphere.h>
#include <OpenSim/version.h>

#include "Model.h"
#include "ModelVisualizer.h"
#include "MarkerSet.h"
#include "Muscle.h"
#include "BodySet.h"

#include <string>
using std::string;
#include <iostream>
using std::cout; using std::cerr; using std::clog; using std::endl;
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
    OpenSimInputListener(ModelVisualizer& viz) : _viz(viz) {}

    /* This is the implementation of the InputListener interface. We're going
    to override only the menu-pick method and ignore anything we don't
    recognize. Caution: this is being called from the Visualizer's input thread,
    *not* the main execution thread. Synchronization is required to do anything
    complicated; here we're just setting/clearing visualization flags so no 
    synchronization is required. Note that the Visualizer's InputSilo class 
    handles synchronization automatically but this one does not. */
    /*virtual*/ bool menuSelected(int menu, int item) {
        if (menu != ShowMenuId) return false; // some other menu
        switch(item) {
        case ToggleWrapGeometry:
            _viz.setShowWrapGeometry(!_viz.getShowWrapGeometry());
            return true; // absorb this input
        case ToggleContactGeometry:
            _viz.setShowContactGeometry(!_viz.getShowContactGeometry());
            return true;
        case ToggleMusclePaths:
            _viz.setShowMusclePaths(!_viz.getShowMusclePaths());
            return true;
        case TogglePathPoints:
            _viz.setShowPathPoints(!_viz.getShowPathPoints());
            return true;
        case ToggleMarkers:
            _viz.setShowMarkers(!_viz.getShowMarkers());
            return true;
        case ToggleDefaultGeometry: {
            SimbodyMatterSubsystem& matter = 
                _viz.updModel().updMatterSubsystem();
            matter.setShowDefaultGeometry(!matter.getShowDefaultGeometry());
            return true;
            }
        };
        return false; // let someone else deal with this input
    }
private:
    ModelVisualizer&    _viz;
};



//==============================================================================
//                           DEFAULT GEOMETRY
//==============================================================================
// This class implements a SimTK DecorationGenerator. We'll add one to the
// Model's DecorationSubsystem so the Visualizer can invoke the 
// generateDecorations() dispatcher to pick up per-frame geometry.
class DefaultGeometry : public DecorationGenerator {
public:
    DefaultGeometry(Model& model) : _model(model) {}
    void generateDecorations(const State& state, 
                             Array_<DecorativeGeometry>& geometry);
private:
    Model& _model;
};

// Draw a path point with a small body-axis-aligned cross centered on
// the point.
static void drawPathPoint(const MobilizedBodyIndex&             body,
                          const Vec3&                           pt_B,
                          const Vec3&                           color,
                          Array_<SimTK::DecorativeGeometry>&    geometry)
{
    const double MarkLen = .005; // half-length of marker line
    const double thickness = 2;

    geometry.push_back(DecorativeLine(pt_B-Vec3(MarkLen,0,0),
                                      pt_B+Vec3(MarkLen,0,0))
                            .setBodyId(body).setColor(color)
                            .setLineThickness(thickness));
    geometry.push_back(DecorativeLine(pt_B-Vec3(0,MarkLen,0),
                                      pt_B+Vec3(0,MarkLen,0))
                            .setBodyId(body).setColor(color)
                            .setLineThickness(thickness));
    geometry.push_back(DecorativeLine(pt_B-Vec3(0,0,MarkLen),
                                      pt_B+Vec3(0,0,MarkLen))
                            .setBodyId(body).setColor(color)
                            .setLineThickness(thickness));
}

void DefaultGeometry::generateDecorations
   (const State&                         state, 
    Array_<SimTK::DecorativeGeometry>&   geometry) 
{
    const SimbodyMatterSubsystem& matter = _model.getMatterSubsystem();
    const ModelVisualizer&        viz    = _model.getModelVisualizer();


    //// Display a cylinder connecting each pair of joints.

    //const JointSet& joints = _model.getJointSet();
    //for (int i = 0; i < joints.getSize(); i++) {
    //    const Joint& joint = joints[i];
    //    if (joint.getType() != "FreeJoint" && joint.getParentBody().hasJoint()) {
    //        const Joint& parent = joint.getParentBody().getJoint();
    //        Vec3 childLocation, parentLocation;
    //        joint.getLocation(childLocation);
    //        parent.getLocation(parentLocation);
    //        childLocation = matter.getMobilizedBody(joint.getBody().getIndex())
    //                                    .getBodyTransform(state)*childLocation;
    //        parentLocation = matter.getMobilizedBody(parent.getBody().getIndex())
    //                                    .getBodyTransform(state)*parentLocation;
    //        double length = (childLocation-parentLocation).norm();
    //        Vec3 center = (childLocation+parentLocation)*0.5;
    //        Rotation orientation;
    //        orientation.setRotationFromOneAxis
    //           (UnitVec3(childLocation-parentLocation), YAxis);
    //        geometry.push_back(DecorativeCylinder(0.1*length, 0.5*length)
    //                            .setTransform(Transform(orientation, center)));
    //    }
    //}

    // Display the path and activation level of each muscle.
    if (viz.getShowMusclePaths()) {
        const Set<Muscle>& muscles = _model.getMuscles();
        for (int i = 0; i < muscles.getSize(); ++i) {
            const Muscle& muscle = muscles[i];
		    const double activation = 
                SimTK::clamp(0., muscle.getActivation(state), 1.);
            const Vec3 color(activation, 0, 1-activation); // blue to red

            _model.updMuscles()[i].updateDisplayer(state);
            const Array<PathPoint*>& points = 
                muscle.getGeometryPath().getCurrentDisplayPath(state);
            if (points.getSize() == 0) 
                continue;

            const PathPoint* lastPoint = points[0];
            Vec3 lastLoc_B = lastPoint->getLocation();
            MobilizedBodyIndex lastBody = lastPoint->getBody().getIndex();

            if (viz.getShowPathPoints())
                drawPathPoint(lastBody, lastLoc_B, 0.9*SimTK::White, geometry);


            Vec3 lastPos = matter.getMobilizedBody(lastBody)
                                .getBodyTransform(state)*lastLoc_B;
            for (int j = 1; j < points.getSize(); j++) {
                const PathPoint* point = points[j];
                const Vec3 loc_B = point->getLocation();
                const MobilizedBodyIndex body = point->getBody().getIndex();
                if (viz.getShowPathPoints())
                    drawPathPoint(body, loc_B, 0.9*SimTK::White, geometry);

                Vec3 pos = matter.getMobilizedBody(body)
                                 .getBodyTransform(state)*loc_B;

                geometry.push_back(DecorativeLine(lastPos, pos)
                                    .setLineThickness(4)
                                    .setColor(color));
                lastPos = pos;
            }
        }
    }

    // Display markers.
    if (viz.getShowMarkers()) {
        const double radius=.005, opacity=1;
        const Vec3 pink(1,.6,.8);
        const MarkerSet& markers = _model.getMarkerSet();
        for (int i=0; i < markers.getSize(); ++i) {
            const Marker& marker = markers[i];
            const OpenSim::Body& body = marker.getBody();
            const Vec3& p_BM = marker.getOffset();
            geometry.push_back(
                DecorativeSphere(radius).setBodyId(body.getIndex())
                .setColor(pink).setOpacity(opacity)
                .setTransform(marker.getOffset()));
        }
    }

    // Display wrap objects.

    if (viz.getShowWrapGeometry()) {
        const double opacity = 0.5;
        const double rez = 2;
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
                const string& type = wrapObjects[j].getType();
                if (type == "WrapCylinder") {
                    const WrapCylinder* cylinder = 
                        dynamic_cast<const WrapCylinder*>(&wrapObjects[j]);
                    if (cylinder != NULL) {
                        Transform X_GW = X_GB*cylinder->getTransform()*ztoy;
                        geometry.push_back(
                            DecorativeCylinder(cylinder->getRadius(), 
                                               cylinder->getLength()/2)
                                .setTransform(X_GW).setResolution(rez)
                                .setColor(color).setOpacity(opacity));
                    }
                }
                else if (type == "WrapEllipsoid") {
                    const WrapEllipsoid* ellipsoid = 
                        dynamic_cast<const WrapEllipsoid*>(&wrapObjects[j]);
                    if (ellipsoid != NULL) {
                        Transform X_GW = X_GB*ellipsoid->getTransform();
                        geometry.push_back(
                            DecorativeEllipsoid(ellipsoid->getRadii())
                                .setTransform(X_GW).setResolution(rez)
                                .setColor(color).setOpacity(opacity));
                    }
                }
                else if (type == "WrapSphere") {
                    const WrapSphere* sphere = 
                        dynamic_cast<const WrapSphere*>(&wrapObjects[j]);
                    if (sphere != NULL) {
                        Transform X_GW = X_GB*sphere->getTransform();
                        geometry.push_back(
                            DecorativeSphere(sphere->getRadius())
                                .setTransform(X_GW).setResolution(rez)
                                .setColor(color).setOpacity(opacity));
                    }
                }
            }
        }
    }
}

//==============================================================================
//                            MODEL VISUALIZER
//==============================================================================

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
            bool isAbsolutePath; string directory, fileName, extension; 
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
// visualization and user interaction. We also rummage through the model to 
// find fixed geometry that should be part of every frame.
void ModelVisualizer::initVisualizer() {
    _model.updDecorationSubsystem().addDecorationGenerator
       (SimTK::Stage::Position, new DefaultGeometry(_model));
    _model.updMatterSubsystem().setShowDefaultGeometry(false);

    // Allocate a Simbody Visualizer.
    _viz = new SimTK::Visualizer(_model.getMultibodySystem());
    _viz->setCameraClippingPlanes(.01,100.);
    _viz->setBackgroundColor(SimTK::Black);
    _viz->setBackgroundType(SimTK::Visualizer::SolidColor);

    // Give it an OpenSim-friendly window heading.
    bool isAbsolutePath; string directory, fileName, extension; 
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

    // Add an input listener to handle display menu picks.
    _viz->addInputListener(new OpenSimInputListener(*this));

    // Allocate an InputSilo to pick up anything the above listener doesn't.
    _silo = new SimTK::Visualizer::InputSilo();
    _viz->addInputListener(_silo);

    // This is used for regular output of frames during forward dynamics.
    _model.updMultibodySystem().addEventReporter
        (new SimTK::Visualizer::Reporter(*_viz, 1./30));


    // Run through all the bodies and try to open the meshes associated
    // with them.
    const BodySet& bodies = _model.getBodySet();
    for (int i=0; i < bodies.getSize(); ++i) {
        const Body& body = bodies[i];
        const MobilizedBodyIndex bx = body.getIndex();
        const VisibleObject& visible = *body.getDisplayer();
        Vec3 scale; visible.getScaleFactors(scale);
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
                default: assert(!"bad DisplayPreference");
            };

            const std::string& file = geo.getGeometryFile();
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

            //TODO: just for debugging
            //if (foundIt) {
            //    std::clog << "ModelVisualizer found file '" << file
            //        << "'; tried\n";
            //    for (unsigned i=0; i < attempts.size(); ++i)
            //        std::clog << "  " << attempts[i] << "\n";
            //}

            SimTK::PolygonalMesh pmesh;
            try {
                if (lowerExtension == ".vtp") {
                    pmesh.loadVtpFile(attempts.back());
                } else {
                    std::ifstream objFile(attempts.back());
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
            // TODO: xyz factors
            dmesh.setScale(geo.getScaleFactors()[0]*scale[0]); 
            _model.updDecorationSubsystem().addBodyFixedDecoration
                (bx, X_BV*geo.getTransform(), dmesh);
        }
    }

}