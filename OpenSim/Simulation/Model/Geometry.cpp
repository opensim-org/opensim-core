/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Geometry.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
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
#include "RigidFrame.h"
#include "Geometry.h"
#include "Model.h"
#include "ModelVisualizer.h"
//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;

/**
 * Compute Transform of a Geometry w.r.t. passed in Frame
 * Both Frame(s) could be Bodies, state is assumed to be realized ro position
*/
SimTK::Transform OpenSim::Geometry::getTransform(const SimTK::State& state, const OpenSim::RigidFrame& frame) const {
    const OpenSim::Model& model = frame.getModel();
    const OpenSim::Frame& gFrame = model.getBodySet().contains(get_frame_name()) ? model.getBodySet().get(get_frame_name()) :
        model.getFrameSet().get(get_frame_name());
    if (model.getBodySet().contains(frame.getName()))
        return  gFrame.calcTransformToOtherFrame(state, model.getBodySet().get(frame.getName()));
    return gFrame.calcTransformToOtherFrame(state, frame);
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
    DecorativeCylinder deco(get_radius(), get_height()/2.);
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

void Ellipsoid::createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const
{
    const Vec3 netScale = get_scale_factors();
    DecorativeEllipsoid deco(get_radii());
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
    bool foundIt = false;// ModelVisualizer::findGeometryFile(file, isAbsolutePath, attempts);

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
