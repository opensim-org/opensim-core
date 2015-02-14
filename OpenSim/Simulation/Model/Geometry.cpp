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
    constructConnector<PhysicalFrame>("frame");
}

void Geometry::setFrameName(const std::string& name)
{
    updConnector<PhysicalFrame>("frame").set_connected_to_name(name);
}
const std::string& Geometry::getFrameName() const
{
    return getConnector<PhysicalFrame>("frame").get_connected_to_name();
}

const OpenSim::PhysicalFrame& Geometry::getFrame() const
{
    return getConnector<PhysicalFrame>("frame").getConnectee();
}

bool Geometry::isFrameSpecified() const {
    return getConnector<OpenSim::PhysicalFrame>("frame").isConnected();
}

void Geometry::generateDecorations(bool fixed, const ModelDisplayHints& hints, const SimTK::State& state,
    SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const
{
    if (!fixed) return; // serialized Geometry is assumed fixed
    SimTK::Array_<SimTK::DecorativeGeometry> decos;
    createDecorativeGeometry(decos);
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
    const PhysicalFrame& myFrame = getFrame();
    const Frame& bFrame = myFrame.findBaseFrame();
    const PhysicalFrame* bPhysicalFrame = dynamic_cast<const PhysicalFrame*>(&bFrame);
    if (bPhysicalFrame == nullptr){
        // throw exception something is wrong
    }
    const SimTK::MobilizedBodyIndex& idx = bPhysicalFrame->getMobilizedBodyIndex();
    SimTK::Transform transformInBaseFrame = myFrame.findTransformInBaseFrame();
    for (unsigned i = 0; i < decorations.size(); i++){
        decorations[i].setBodyId(idx);
        decorations[i].setTransform(transformInBaseFrame);
    }
}
void Sphere::createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const Vec3 netScale = get_scale_factors();
    DecorativeSphere deco(get_radius());
    deco.setScaleFactors(netScale);
    decoGeoms.push_back(deco);
}

void Cylinder::createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const Vec3 netScale = get_scale_factors();
    DecorativeCylinder deco(get_radius(), get_half_height());
    deco.setScaleFactors(netScale);
    decoGeoms.push_back(deco);
}

void LineGeometry::createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const Vec3 netScale = get_scale_factors();
    DecorativeLine deco(get_start_point(), get_end_point());
    deco.setScaleFactors(netScale);
    decoGeoms.push_back(deco);
}

void ArrowGeometry::createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    LineGeometry::createDecorativeGeometry(decoGeoms);
    // Add Brick at the top
    DecorativeBrick tip(Vec3(.005));
    tip.setTransform(SimTK::Transform(get_end_point()));
    decoGeoms.push_back(tip);
}

void Ellipsoid::createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const Vec3 netScale = get_scale_factors();
    DecorativeEllipsoid deco(get_radii());
    deco.setScaleFactors(netScale);
    decoGeoms.push_back(deco);
}

void Brick::createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const Vec3 netScale = get_scale_factors();
    DecorativeBrick deco(get_half_lengths());
    deco.setScaleFactors(netScale);
    decoGeoms.push_back(deco);
}

void Mesh::createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const std::string& file = get_mesh_file();
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

    DecorativeMesh dmesh(pmesh);
    dmesh.setScaleFactors(get_scale_factors());
    decoGeoms.push_back(dmesh);
}
