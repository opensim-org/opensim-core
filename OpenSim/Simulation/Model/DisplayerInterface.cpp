/* -------------------------------------------------------------------------- *
 *                             OpenSim:  DisplayerInterface.cpp               *
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


// INCLUDE
#include "Simbody.h"
#include "DisplayerInterface.h"
#include "ModelComponent.h"
#include "RigidFrame.h"
#include "ModelVisualizer.h"

using namespace OpenSim;
using namespace std;
using SimTK::Vec3;
using SimTK::MobilizedBodyIndex;
using SimTK::DecorativeGeometry;
using SimTK::DecorativeLine;
using SimTK::DefaultGeometry;

void DefaultDisplayer::generateDecorations(const ModelComponent& mc,
    bool fixed,
    const ModelDisplayHints&                    hints,
    const SimTK::State&                         state,
    SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const
{
    const OpenSim::RigidFrame* frm = dynamic_cast<const OpenSim::RigidFrame*>(&mc);
    if (frm != nullptr)
        generateDecorationsInFrame(*frm, fixed, hints, state, appendToThis);
    else
        generateDecorationsNeedFrame(mc, fixed, hints, state, appendToThis);
}

void DefaultDisplayer::generateDecorationsInFrame(const RigidFrame& frame,
    bool fixed,
    const ModelDisplayHints&                    hints,
    const SimTK::State&                         state,
    SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const
{
    SimTK::MobilizedBodyIndex bx = frame.getMobilizedBodyIndex();
    int nGeom = frame.getProperty_GeometrySet().size();
    const Model& model = frame.getModel();
    for (int g = 0; g < nGeom; ++g) {
        const Geometry& geo = frame.get_GeometrySet(g);
         //
        SimTK::Transform xformRelativeToBody;
        xformRelativeToBody = geo.getTransform(state, frame);
        const OpenSim::Mesh* mGeom = Mesh::safeDownCast(const_cast<OpenSim::Geometry*>(&geo));
        if (mGeom){
            const std::string& file = mGeom->get_mesh_file();
            bool isAbsolutePath; string directory, fileName, extension;
            SimTK::Pathname::deconstructPathname(file,
                isAbsolutePath, directory, fileName, extension);
            const string lowerExtension = SimTK::String::toLower(extension);
            if (lowerExtension != ".vtp" && lowerExtension != ".obj") {
                std::clog << "ModelVisualizer ignoring '" << file
                    << "'; only .vtp and .obj files currently supported.\n";
                continue;
            }

            // File is a .vtp or .obj. See if we can find it.
            SimTK::Array_<string> attempts;
            bool foundIt = ModelVisualizer::findGeometryFile(model, file, isAbsolutePath, attempts);

            if (!foundIt) {
                std::clog << "ModelVisualizer couldn't find file '" << file
                    << "'; tried\n";
                for (unsigned i = 0; i < attempts.size(); ++i)
                    std::clog << "  " << attempts[i] << "\n";
                if (!isAbsolutePath &&
                    !SimTK::Pathname::environmentVariableExists("OPENSIM_HOME"))
                    std::clog << "Set environment variable OPENSIM_HOME "
                    << "to search $OPENSIM_HOME/Geometry.\n";
                continue;
            }

            SimTK::DecorativeMeshFile dmesh(file);
            const Vec3 netScale = geo.get_scale_factors();
            dmesh.setScaleFactors(netScale);
            dmesh.setTransform(xformRelativeToBody);
            geo.setDecorativeGeometryAppearance(dmesh);
            dmesh.setBodyId(bx);
            dmesh.setIndexOnBody(g + 1);
            appendToThis.push_back(dmesh);
        }
        else {
            SimTK::Array_<SimTK::DecorativeGeometry> deocrationsForGeom;
            geo.createDecorativeGeometry(deocrationsForGeom);
            for (unsigned gi = 0; gi < deocrationsForGeom.size(); ++gi){
                SimTK::DecorativeGeometry dg = deocrationsForGeom[gi];
                dg.setTransform(xformRelativeToBody);
                dg.setBodyId(bx);
                dg.setIndexOnBody(100*g+gi + 1);
                geo.setDecorativeGeometryAppearance(dg);
                appendToThis.push_back(dg);
            }
        }
    }
}
void DefaultDisplayer::generateDecorationsNeedFrame(const ModelComponent& mc,
    bool fixed,
    const ModelDisplayHints&                    hints,
    const SimTK::State&                         state,
    SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const
{
        int nGeom = mc.getProperty_GeometrySet().size();
        const Model& model = mc.getModel();
        for (int g = 0; g < nGeom; ++g) {
            const Geometry& geo = mc.get_GeometrySet(g);
            const Vec3 netScale = geo.get_scale_factors();
            SimTK::Transform xformRelativeToBody;
            const std::string& frameName = geo.isFrameSpecified()?geo.get_frame_name():"ground";
            ComponentList<RigidFrame> framesList = model.getComponentList<OpenSim::RigidFrame>();
            const RigidFrame* frame=0;
            for (const RigidFrame& frm : framesList) {
                if (frm.getName() == frameName){
                    frame = &frm;
                    break;
                }
            }
            xformRelativeToBody = geo.getTransform(state, *frame);

            const OpenSim::Mesh* mGeom = Mesh::safeDownCast(const_cast<OpenSim::Geometry*>(&geo));
            if (mGeom){
                const std::string& file = mGeom->get_mesh_file();
                bool isAbsolutePath; string directory, fileName, extension;
                SimTK::Pathname::deconstructPathname(file,
                    isAbsolutePath, directory, fileName, extension);
                const string lowerExtension = SimTK::String::toLower(extension);
                if (lowerExtension != ".vtp" && lowerExtension != ".obj") {
                    std::clog << "ModelVisualizer ignoring '" << file
                        << "'; only .vtp and .obj files currently supported.\n";
                    continue;
                }

                // File is a .vtp or .obj. See if we can find it.
                SimTK::Array_<string> attempts;
                bool foundIt = ModelVisualizer::findGeometryFile(model, file, isAbsolutePath, attempts);

                if (!foundIt) {
                    std::clog << "ModelVisualizer couldn't find file '" << file
                        << "'; tried\n";
                    for (unsigned i = 0; i < attempts.size(); ++i)
                        std::clog << "  " << attempts[i] << "\n";
                    if (!isAbsolutePath &&
                        !SimTK::Pathname::environmentVariableExists("OPENSIM_HOME"))
                        std::clog << "Set environment variable OPENSIM_HOME "
                        << "to search $OPENSIM_HOME/Geometry.\n";
                    continue;
                }

                SimTK::DecorativeMeshFile dmesh(file);
                dmesh.setScaleFactors(netScale);
                dmesh.setTransform(xformRelativeToBody);
                geo.setDecorativeGeometryAppearance(dmesh);
                dmesh.setBodyId(frame->getMobilizedBodyIndex());
                dmesh.setIndexOnBody(g + 1);
                appendToThis.push_back(dmesh);
            }
            else {
                SimTK::Array_<SimTK::DecorativeGeometry> deocrationsForGeom;
                geo.createDecorativeGeometry(deocrationsForGeom);
                for (unsigned gi = 0; gi < deocrationsForGeom.size(); ++gi){
                    SimTK::DecorativeGeometry dg = deocrationsForGeom[gi];
                    dg.setTransform(xformRelativeToBody);
                    dg.setBodyId(frame->getMobilizedBodyIndex());
                    dg.setIndexOnBody(100*g + gi +1);
                    geo.setDecorativeGeometryAppearance(dg);
                    appendToThis.push_back(dg);
                }
            }
        }

}

void PathDisplayer::generateDecorations(const OpenSim::ModelComponent& mc,
    bool fixed,
    const OpenSim::ModelDisplayHints&           hints,
    const SimTK::State&                         state,
    SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const
{
    DefaultDisplayer::generateDecorations(mc, fixed, hints, state, appendToThis);
    if (fixed) return;

    const OpenSim::GeometryPath& gp = dynamic_cast<const OpenSim::GeometryPath&>(mc);
    const Array<PathPoint*>& points = gp.getCurrentDisplayPath(state);

    if (points.getSize() == 0) { return; }

    const PathPoint* lastPoint = points[0];
    Vec3 lastLoc_B = lastPoint->getLocation();
    MobilizedBodyIndex lastBody = lastPoint->getBody().getMobilizedBodyIndex();

    if (hints.get_show_path_points())
        DefaultGeometry::drawPathPoint(lastBody, lastLoc_B, gp.getColor(state),
        appendToThis);

    const SimTK::SimbodyMatterSubsystem& matter = gp.getModel().getMatterSubsystem();
    Vec3 lastPos = matter.getMobilizedBody(lastBody)
        .getBodyTransform(state) * lastLoc_B;

    for (int j = 1; j < points.getSize(); j++) {
        const PathPoint* point = points[j];
        const Vec3 loc_B = point->getLocation();
        const MobilizedBodyIndex body = point->getBody().getMobilizedBodyIndex();

        if (hints.get_show_path_points())
            DefaultGeometry::drawPathPoint(body, loc_B, gp.getColor(state),
            appendToThis);

        Vec3 pos = matter.getMobilizedBody(body).getBodyTransform(state)*loc_B;
        // Line segments will be in ground frame
        appendToThis.push_back(DecorativeLine(lastPos, pos)
            .setLineThickness(4)
            .setColor(gp.getColor(state)).setBodyId(0).setIndexOnBody(j));

        lastPos = pos;
    }

}
