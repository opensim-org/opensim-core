/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Geometry.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <fstream>
#include "Frame.h"
#include "Geometry.h"
#include "Model.h"
//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;

OpenSim_DEFINE_SOCKET_FD(frame, Geometry);

Geometry::Geometry() {
    setNull();
    constructProperties();
}

void Geometry::setFrame(const Frame& frame)
{
    updSocket<Frame>("frame").setConnecteePath(frame.getRelativePathString(*this));
}

const OpenSim::Frame& Geometry::getFrame() const
{
    return getSocket<Frame>("frame").getConnectee();
}

void Geometry::extendFinalizeConnections(Component& root)
{
    Super::extendFinalizeConnections(root);

    bool attachedToFrame = getSocket<Frame>("frame").isConnected();
    bool hasInputTransform = getInput("transform").isConnected();
    // Being both attached to a Frame (i.e. Socket<Frame> connected) 
    // and the Input transform connected has ambiguous behavior so disallow it
    if (attachedToFrame && hasInputTransform ) {
        OPENSIM_THROW(Exception, getConcreteClassName() + " '" + getName()
            + "' cannot be attached to a Frame and have its "
                "Input `transform` set.");
    }
    else if (!attachedToFrame && !hasInputTransform) {
        OPENSIM_THROW(Exception, getConcreteClassName() + " '" + getName()
            + "' must be attached to a Frame OR have its "
                "Input `transform` set.");
    }
}

void FrameGeometry::generateDecorations(bool fixed,
    const ModelDisplayHints& hints,
    const SimTK::State& state,
    SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const
{
    if (!hints.get_show_frames())
        return;
    // Call base class
    Super::generateDecorations(fixed, hints, state, appendToThis);

}
void Geometry::generateDecorations(bool fixed, 
    const ModelDisplayHints& hints,
    const SimTK::State& state,
    SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const
{
    // serialized Geometry is assumed fixed
    // if it has a Transform input then it is not "attached" geometry
    // and fixed to a body but floating w.r.t Ground.
    if (!fixed && !getInput("transform").isConnected())
        return; 
    
    if (!get_Appearance().get_visible()) return;

    SimTK::Array_<SimTK::DecorativeGeometry> decos;
    implementCreateDecorativeGeometry(decos);
    if (decos.size() == 0) return;
    setDecorativeGeometryTransform(decos, state);
    for (unsigned i = 0; i < decos.size(); i++){
        setDecorativeGeometryAppearance(decos[i]);
        appendToThis.push_back(decos[i]);
    }
}

/*
 * Apply the Transform of the Frame the Geometry is attached to,
 * OR use the Transform supplied to the Geometry via its Input.
*/
void Geometry::setDecorativeGeometryTransform(
    SimTK::Array_<SimTK::DecorativeGeometry>& decorations, 
    const SimTK::State& state) const
{
    auto& input = getInput<SimTK::Transform>("transform");

    SimTK::Transform transformInBaseFrame;
    SimTK::MobilizedBodyIndex mbidx;

    if (input.isConnected()) {
        transformInBaseFrame = input.getValue(state);
        mbidx = SimTK::MobilizedBodyIndex(0);
    }
    else {
        const Frame& myFrame = getFrame();
        const Frame& bFrame = myFrame.findBaseFrame();
        const PhysicalFrame* bPhysicalFrame =
            dynamic_cast<const PhysicalFrame*>(&bFrame);
        if (bPhysicalFrame == nullptr) {
            // throw exception something is wrong
            throw (Exception("Frame for Geometry " + getName() +
                " is not attached to a PhysicalFrame."));
        }
        mbidx = bPhysicalFrame->getMobilizedBodyIndex();
        transformInBaseFrame = myFrame.findTransformInBaseFrame();
    }

    for (unsigned i = 0; i < decorations.size(); i++){
        decorations[i].setBodyId(mbidx);
        decorations[i].setTransform(transformInBaseFrame);
        decorations[i].setIndexOnBody(i);
    }
}
void Sphere::implementCreateDecorativeGeometry(
    SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const Vec3 netScale = get_scale_factors();
    DecorativeSphere deco(get_radius());
    deco.setScaleFactors(netScale);
    decoGeoms.push_back(deco);
}

void Cylinder::implementCreateDecorativeGeometry(
    SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const Vec3 netScale = get_scale_factors();
    DecorativeCylinder deco(get_radius(), get_half_height());
    deco.setScaleFactors(netScale);
    decoGeoms.push_back(deco);
}

void Cone::implementCreateDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const Vec3 netScale = get_scale_factors();
    DecorativeCone deco(get_origin(), SimTK::UnitVec3(get_direction()), get_height(), get_base_radius());
    deco.setScaleFactors(netScale);
    decoGeoms.push_back(deco);
}

void LineGeometry::implementCreateDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const Vec3 netScale = get_scale_factors();
    DecorativeLine deco(get_start_point(), get_end_point());
    deco.setScaleFactors(netScale);
    decoGeoms.push_back(deco);
}


void Arrow::implementCreateDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const Vec3 netScale = get_scale_factors();

    const Vec3 start = get_start_point();
    const Vec3 end = start + get_length()*get_direction().normalize();

    DecorativeArrow deco(start, end);
    deco.setLineThickness(0.05);
    deco.setScaleFactors(netScale);
    decoGeoms.push_back(deco);
}

void Ellipsoid::implementCreateDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const Vec3 netScale = get_scale_factors();
    DecorativeEllipsoid deco(get_radii());
    deco.setScaleFactors(netScale);
    decoGeoms.push_back(deco);
}

void Brick::implementCreateDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const Vec3 netScale = get_scale_factors();
    DecorativeBrick deco(get_half_lengths());
    deco.setScaleFactors(netScale);
    decoGeoms.push_back(deco);
}

void FrameGeometry::implementCreateDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const Vec3 netScale = get_scale_factors();
    DecorativeFrame deco(1.0);
    deco.setLineThickness(get_display_radius());
    deco.setScaleFactors(netScale);
    decoGeoms.push_back(deco);
}

void Mesh::extendFinalizeFromProperties() {

    if (!isObjectUpToDateWithProperties()) {
        const Component* rootModel = nullptr;
        if (!hasOwner()) {
            log_error("Mesh {} not connected to model...ignoring",
                    get_mesh_file());
            return;   // Orphan Mesh not part of a model yet
        }
        const Component* owner = &getOwner();
        while (owner != nullptr) {
            if (dynamic_cast<const Model*>(owner) != nullptr) {
                rootModel = owner;
                break;
            }
            if (owner->hasOwner())
                owner = &(owner->getOwner()); // traverse up Component tree
            else
                break; // can't traverse up.
        }

        if (rootModel == nullptr) {
            log_error("Mesh {} not connected to model...ignoring",
                    get_mesh_file());
            return;   // Orphan Mesh not descendant of a model
        }

        // Current interface to Visualizer calls generateDecorations on every
        // frame. On first time through, load file and create DecorativeMeshFile
        // and cache it so we don't load files from disk during live rendering.
        const Model* mdl = dynamic_cast<const Model*>(rootModel);
        const std::string& file = get_mesh_file();
        if (file.empty() || file.compare(PropertyStr::getDefaultStr()) == 0 ||
            !mdl->getDisplayHints().isVisualizationEnabled())
            return;  // Return immediately if no file has been specified
                     // or display is disabled altogether.

        bool isAbsolutePath; string directory, fileName, extension;
        SimTK::Pathname::deconstructPathname(file,
            isAbsolutePath, directory, fileName, extension);
        const string lowerExtension = SimTK::String::toLower(extension);
        if (lowerExtension != ".vtp" && lowerExtension != ".obj" && lowerExtension != ".stl") {
            log_error("ModelVisualizer ignoring '{}'; only .vtp, .stl, and "
                      ".obj files currently supported.",
                    file);
            return;
        }

        // File is a .vtp, .stl, or .obj; attempt to find it.
        Array_<string> attempts;
        const Model& model = dynamic_cast<const Model&>(*rootModel);
        bool foundIt = ModelVisualizer::findGeometryFile(model, file, isAbsolutePath, attempts);

        if (!foundIt) {
            if (!warningGiven) {
                log_warn("Couldn't find file '{}'.", file);
                warningGiven = true;
            }
            
            log_debug( "The following locations were tried:");
            for (unsigned i = 0; i < attempts.size(); ++i)
                log_debug(attempts[i]);
            
        }

        try {
            std::ifstream objFile;
            objFile.open(attempts.back().c_str());
            // objFile closes when destructed
            // if the file can be opened but had bad contents e.g. binary vtp 
            // it will be handled downstream 
        }
        catch (const std::exception& e) {
            log_warn("Visualizer couldn't open {} because: {}",
                attempts.back(), e.what());
            return;
        }

        cachedMesh.reset(new DecorativeMeshFile(attempts.back().c_str()));
    }
}


void Mesh::implementCreateDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    if (cachedMesh.get() != nullptr) {
        try {
            // Force the loading of the mesh to see if it has bad contents
            // (e.g., binary vtp).
            // We do not want to do this in extendFinalizeFromProperties b/c
            // it's expensive to repeatedly load meshes.
            cachedMesh->getMesh();
        } catch (const std::exception& e) {
            log_warn("Visualizer couldn't open {} because: {}",
                get_mesh_file(), e.what());
            // No longer try to visualize this mesh.
            cachedMesh.reset();
            return;
        }
        cachedMesh->setScaleFactors(get_scale_factors());
        decoGeoms.push_back(*cachedMesh);
    }
}
