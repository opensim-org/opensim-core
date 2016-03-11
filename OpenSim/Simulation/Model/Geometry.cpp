/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Geometry.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Property.h>
#include <OpenSim/Common/Component.h>
#include "Frame.h"
#include "PhysicalFrame.h"
#include "Geometry.h"
#include "Model.h"
#include "ModelVisualizer.h"
//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;


void Geometry::constructConnectors()
{
    constructConnector<Frame>("frame");
}

void Geometry::setFrameName(const std::string& name)
{
    updConnector<Frame>("frame").set_connectee_name(name);
}

void Geometry::setFrame(const Frame& frame)
{
    updConnector<Frame>("frame").set_connectee_name(frame.getRelativePathName(*this));
}


const std::string& Geometry::getFrameName() const
{
    return getConnector<Frame>("frame").get_connectee_name();
}

const OpenSim::Frame& Geometry::getFrame() const
{
    return getConnector<Frame>("frame").getConnectee();
}

void Geometry::generateDecorations(bool fixed, const ModelDisplayHints& hints, const SimTK::State& state,
    SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const
{
    if (!fixed) return; // serialized Geometry is assumed fixed
    SimTK::Array_<SimTK::DecorativeGeometry> decos;
    implementCreateDecorativeGeometry(decos);
    if (decos.size() == 0) return;
    setDecorativeGeometryTransform(decos, state);
    for (unsigned i = 0; i < decos.size(); i++){
        setDecorativeGeometryAppearance(decos[i]);
        appendToThis.push_back(decos[i]);
    }
}

/**
 * Compute Transform of a Geometry w.r.t. passed in Frame
 * Both Frame(s) could be Bodies, state is assumed to be realized to position
*/
void Geometry::setDecorativeGeometryTransform(SimTK::Array_<SimTK::DecorativeGeometry>& decorations, const SimTK::State& state) const
{
    const Frame& myFrame = getFrame();
    const Frame& bFrame = myFrame.findBaseFrame();
    const PhysicalFrame* bPhysicalFrame = dynamic_cast<const PhysicalFrame*>(&bFrame);
    if (bPhysicalFrame == nullptr){
        // throw exception something is wrong
        throw (Exception("Frame for Geometry " + getName() + " is not attached to a PhysicalFrame."));
    }
    const SimTK::MobilizedBodyIndex& idx = bPhysicalFrame->getMobilizedBodyIndex();
    SimTK::Transform transformInBaseFrame = myFrame.findTransformInBaseFrame();
    for (unsigned i = 0; i < decorations.size(); i++){
        decorations[i].setBodyId(idx);
        decorations[i].setTransform(transformInBaseFrame);
        decorations[i].setIndexOnBody(i);
    }
}
void Sphere::implementCreateDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const Vec3 netScale = get_scale_factors();
    DecorativeSphere deco(get_radius());
    deco.setScaleFactors(netScale);
    decoGeoms.push_back(deco);
}

void Cylinder::implementCreateDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
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
    SimTK::Vec3 endPt(get_length()*get_direction());
    DecorativeArrow deco(SimTK::Vec3(0), endPt);
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

void Mesh::implementCreateDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const std::string& file = get_mesh_file();
    // TODO: when API visualizer changes to use DecorativeGeometry::MeshFile instead of 
    // DecorativeGeometry::DecorativeMesh with PolygonalMesh underneath it, the logic below 
    // to locate the files will need to be transferred there. -Ayman 05/15
#if 0
    bool isAbsolutePath; string directory, fileName, extension;
    SimTK::Pathname::deconstructPathname(file,
        isAbsolutePath, directory, fileName, extension);
    const string lowerExtension = SimTK::String::toLower(extension);
    if (lowerExtension != ".vtp" && lowerExtension != ".obj") {
        std::clog << "ModelVisualizer ignoring '" << file
            << "'; only .vtp and .obj files currently supported.\n";
        return;
    }

    // File is a .vtp or .obj. See if we can find it.
    Array_<string> attempts;
    bool foundIt = ModelVisualizer::findGeometryFile(getFrame().getModel(), file, isAbsolutePath, attempts);

    if (!foundIt) {
        std::clog << "ModelVisualizer couldn't find file '" << file
            << "'; tried\n";
        for (unsigned i = 0; i < attempts.size(); ++i)
            std::clog << "  " << attempts[i] << "\n";
        if (!isAbsolutePath &&
            !Pathname::environmentVariableExists("OPENSIM_HOME"))
            std::clog << "Set environment variable OPENSIM_HOME "
            << "to search $OPENSIM_HOME/Geometry.\n";
        return;
    }

    SimTK::PolygonalMesh pmesh;
    try {
        if (lowerExtension == ".vtp") {
            pmesh.loadVtpFile(attempts.back());
        }
        else {
            std::ifstream objFile;
            objFile.open(attempts.back().c_str());
            pmesh.loadObjFile(objFile);
            // objFile closes when destructed
        }
    }
    catch (const std::exception& e) {
        std::clog << "ModelVisualizer couldn't read "
            << attempts.back() << " because:\n"
            << e.what() << "\n";
        return;
    }
#endif
    DecorativeMeshFile dmesh(file);
    dmesh.setScaleFactors(get_scale_factors());
    decoGeoms.push_back(dmesh);
}
