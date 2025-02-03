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
#include "Geometry.h"

#include "Frame.h"
#include "Model.h"

#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <utility>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;

OpenSim_DEFINE_SOCKET_FD(frame, Geometry);

namespace
{
    // Returns a pointer to `c`'s owner, or `nullptr` if `c` does not have an owner.
    const OpenSim::Component* tryGetOwner(const OpenSim::Component& c)
    {
        return c.hasOwner() ? &c.getOwner() : nullptr;
    }

    // Returns a pointer to the closest ancestor of `c` that has type `T`, or
    // `nullptr` if no such owner exists.
    template<typename T>
    const T* findFirstOwnerOfType(const OpenSim::Component& c)
    {
        for (const OpenSim::Component* cur = tryGetOwner(c); cur; cur = tryGetOwner(*cur)) {
            if (const T* downcasted = dynamic_cast<const T*>(cur)) {
                return downcasted;
            }
        }
        return nullptr;
    }

    // Returns `true` if `str` has a suffix of `suffix`, ignoring case.
    bool hasSuffixCaseInsensitive(const std::string& str, const std::string& suffix)
    {
        if (str.size() < suffix.size()) {
            return false;
        }
        for (std::string::size_type i = 0; i < suffix.size(); ++i) {
            if (std::tolower(str.rbegin()[i]) != std::tolower(suffix.rbegin()[i])) {
                return false;
            }
        }
        return true;
    }

    // Returns an absolute path to the underlying geometry file that can be
    // associated with `file`; otherwise, returns `std::nullopt`.
    //
    // Prints search errors to the log if `warningGiven` is `false` and then
    // flips `warningGiven` to `true` (i.e. it's a one-time flag).
    std::optional<std::string> findGeometry(
        const OpenSim::Model& model,
        const std::string& file,
        bool& warningGiven)
    {
        Array_<string> attempts;
        bool isAbsolutePath = false;
        if (ModelVisualizer::findGeometryFile(model, file, isAbsolutePath, attempts)) {
            return std::move(attempts.back());
        }

        // Else: geometry file could not be found, print warning
        if (!std::exchange(warningGiven, true)) {
            log_warn("Couldn't find file '{}'.", file);
            log_debug( "The following locations were tried:");
            for (const auto& attempt : attempts) {
                log_debug(attempt);
            }
        }
        return std::nullopt;
    }
}

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

// Internal implementation of a cached mesh file that uses the file's
// modification timestamp to figure out whether the mesh was modified.
class OpenSim::Mesh::CachedDecorativeMeshFile final {
public:
    explicit CachedDecorativeMeshFile(
        const std::filesystem::path& meshAbsPath,
        const SimTK::Vec3& scaleFactors) :

        _meshFileModificationTime{std::filesystem::last_write_time(meshAbsPath)},
        _meshFile{meshAbsPath.string()}
    {
        _meshFile.getMesh();  // Eagerly load mesh data
        _meshFile.setScaleFactors(scaleFactors);
    }

    const std::string& getMeshFilePath() const { return _meshFile.getMeshFile(); }
    const std::filesystem::file_time_type& getModificationTime() const { return _meshFileModificationTime; }
    const SimTK::Vec3& getScaleFactors() const { return _meshFile.getScaleFactors(); }
    void setScaleFactors(const SimTK::Vec3& newScaleFactors) { _meshFile.setScaleFactors(newScaleFactors); }
    const SimTK::DecorativeGeometry& getGeometry() const { return _meshFile; }
private:
    std::filesystem::file_time_type _meshFileModificationTime;
    SimTK::DecorativeMeshFile _meshFile;
};

Mesh::Mesh()
{
    constructProperty_mesh_file("");
}

Mesh::Mesh(const std::string& geomFile)
{
    constructProperty_mesh_file("");
    upd_mesh_file() = geomFile;
}

void Mesh::extendFinalizeFromProperties()
{
    if (isObjectUpToDateWithProperties()) {
        return;  // No need to re-finalize.
    }

    const std::string& meshPath = get_mesh_file();
    if (meshPath.empty() || meshPath == PropertyStr::getDefaultStr()) {
        _mesh.reset();
        return;  // No mesh specified.
    }

    if (!(hasSuffixCaseInsensitive(meshPath, ".vtp") ||
          hasSuffixCaseInsensitive(meshPath, ".obj") ||
          hasSuffixCaseInsensitive(meshPath, ".stl"))) {

        log_error("ModelVisualizer ignoring '{}'; only .vtp, .stl, and .obj files currently supported.", meshPath);
        _mesh.reset();
        return;  // Unsupported file format.
    }

    const auto* model = findFirstOwnerOfType<OpenSim::Model>(*this);
    if (!model) {
        log_error("Mesh {} not connected to model...ignoring", get_mesh_file());
        _mesh.reset();
        return;  // This component isn't connected to a model.
    }

    if (!model->getDisplayHints().isVisualizationEnabled()) {
        _mesh.reset();
        return;  // Visualization is disabled.
    }

    const std::optional<std::string> meshAbsPath = findGeometry(*model, meshPath, _warningGiven);
    if (!meshAbsPath) {
        _mesh.reset();
        return;  // Couldn't find the mesh.
    }

    // Completely reset the cached mesh if the underlying filepath/modification
    // time has changed.
    if (_mesh &&
        (_mesh->getMeshFilePath() != *meshAbsPath ||
         std::filesystem::last_write_time(*meshAbsPath) != _mesh->getModificationTime())) {

        _mesh.reset();
    }

    if (!_mesh) {
        // There is no cached mesh, load a new one from scratch.
        try {
            _mesh = std::make_shared<CachedDecorativeMeshFile>(*meshAbsPath, get_scale_factors());
        }
        catch (const std::exception& ex) {
            log_warn("Visualizer couldn't open {} because: {}", get_mesh_file(), ex.what());
        }
    }
    else if (_mesh->getScaleFactors() != get_scale_factors()) {
        // There is a cached mesh, but it has invalid scale factors, copy the mesh
        // data, update the scale factors, but don't reload from the filesystem.
        auto meshCopy = std::make_shared<CachedDecorativeMeshFile>(*_mesh);
        meshCopy->setScaleFactors(get_scale_factors());
        _mesh = std::move(meshCopy);
    }
}

void Mesh::implementCreateDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    if (_mesh) {
        decoGeoms.push_back(_mesh->getGeometry());
    }
}
