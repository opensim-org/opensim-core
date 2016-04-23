/* --------------------------------------------------------------------------*
*                         OpenSim:  PhysicalFrame.cpp                        *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2015 Stanford University and the Authors                *
* Author(s): Matt DeMers, Ayman Habib, Ajay Seth                             *
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
#include "PhysicalFrame.h"
#include "Model.h"


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
* Default constructor.
*/
PhysicalFrame::PhysicalFrame() : Frame()
{
    setAuthors("Matt DeMers, Ayman Habib, Ajay Seth");
    constructProperties();
}

void PhysicalFrame::constructProperties()
{
    constructProperty_WrapObjectSet(WrapObjectSet());

    FrameGeometry frm;
    frm.setName("frame_geometry");
    frm.upd_Appearance().set_visible(false);
    updProperty_geometry().setValue(0, frm);
}

void PhysicalFrame::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();
    if (!getProperty_geometry().empty()) {
        upd_geometry(0).setFrame(*this);
    }
}

const SimTK::MobilizedBody& PhysicalFrame::getMobilizedBody() const
{
    return getModel().getMatterSubsystem().getMobilizedBody(_mbIndex);
}

SimTK::MobilizedBody& PhysicalFrame::updMobilizedBody() 
{
    return updModel().updMatterSubsystem().updMobilizedBody(_mbIndex);
}

/*
* Implementation of Frame interface by PhysicalFrame.
* 
*/
SimTK::Transform PhysicalFrame::
    calcGroundTransform(const SimTK::State& s) const
{
    // return X_GF = X_GB * X_BF;
    return getMobilizedBody().getBodyTransform(s);
}

SimTK::Transform PhysicalFrame::extendFindTransformInBaseFrame() const
{
    return SimTK::Transform();
}

void PhysicalFrame::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);

    for (int i = 0; i < get_WrapObjectSet().getSize(); i++)
        get_WrapObjectSet().get(i).connectToModelAndBody(aModel, *this);
}

const WrapObject* PhysicalFrame::getWrapObject(const string& aName) const
{
    int i;

    for (i = 0; i < get_WrapObjectSet().getSize(); i++) {
        if (aName == get_WrapObjectSet()[i].getName())
            return &get_WrapObjectSet()[i];
    }
    return nullptr;
}

void PhysicalFrame::addWrapObject(WrapObject* wrap) {
    upd_WrapObjectSet().adoptAndAppend(wrap);
}

void PhysicalFrame::scale(const SimTK::Vec3& aScaleFactors)
{
    // Base class, to scale wrap objects
    for (int i = 0; i< get_WrapObjectSet().getSize(); i++)
        upd_WrapObjectSet().get(i).scale(aScaleFactors);

    // TODO: When we redo scaling and decide where scale factors
    // are maintained, we may need to fix this or remove this comment completely.
    // -Ayman 5/15
    
    // Scale the Geometry if any
    for (int i = 0; i < getNumGeometry(); ++i){
        const SimTK::Vec3& oldScaleFactor = get_geometry(i).get_scale_factors();
        // Recompute scale factors for Geometry
        SimTK::Vec3 newScaleFactor = oldScaleFactor.elementwiseMultiply(aScaleFactors);
        upd_geometry(i).set_scale_factors(newScaleFactor);
    }
}


//=============================================================================
// XML Deserialization
//=============================================================================

void PhysicalFrame::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    if (versionNumber < XMLDocument::getLatestVersion()) {
        if (versionNumber < 30502) {
            // Find node for <VisibleObject> remove it then create Geometry for the 
            SimTK::Xml::element_iterator iIter = aNode.element_begin("VisibleObject");
            if (iIter != aNode.element_end()) {
                SimTK::Xml::Element visObjElement = SimTK::Xml::Element::getAs(aNode.removeNode(iIter));
                // Scale factors
                SimTK::Vec3 outerScaleFactors(1.0);
                SimTK::Xml::element_iterator outerScaleFactortIter = visObjElement.element_begin("scale_factors");
                if (outerScaleFactortIter != visObjElement.element_end()) {
                    outerScaleFactors = outerScaleFactortIter->getValueAs<SimTK::Vec3>();
                }
                SimTK::Vec6 outerTransform(0.0);
                SimTK::Xml::element_iterator outerTransformIter = visObjElement.element_begin("transform");
                if (outerTransformIter != visObjElement.element_end()) {
                    outerTransform = outerTransformIter->getValueAs<SimTK::Vec6>();
                }

                if (versionNumber < 20101) {
                    SimTK::Xml::element_iterator geometryIter = visObjElement.element_begin("geometry_files");
                    SimTK::Array_<SimTK::String> oldGeometryFiles;
                    if (geometryIter != aNode.element_end()) {
                        geometryIter->getValueAs(oldGeometryFiles);
                    }
                    std::string bodyName = aNode.getRequiredAttribute("name").getValue();
                    SimTK::Xml::Element geometrySetNode("geometry");
                    aNode.insertNodeAfter(aNode.element_end(), geometrySetNode);
                    // Create Mesh node for each item in oldGeometryFiles
                    for (unsigned ng = 0; ng < oldGeometryFiles.size(); ng++) {
                        SimTK::Xml::Element meshNode("Mesh");
                        std::string geomName = bodyName + "_geom_" + to_string(ng);
                        meshNode.setAttributeValue("name", geomName);
                        SimTK::Xml::Element meshFileNode("mesh_file", oldGeometryFiles[ng]);
                        std::stringstream localScaleStr;
                        localScaleStr << outerScaleFactors[0] << " " << outerScaleFactors[1]
                            << " " << outerScaleFactors[2];
                        SimTK::Xml::Element scaleFactorsNode("scale_factors", localScaleStr.str());
                        meshNode.insertNodeAfter(meshNode.element_end(), scaleFactorsNode);
                        meshNode.insertNodeAfter(meshNode.element_end(), meshFileNode);
                        XMLDocument::addConnector(meshNode, "Connector_Frame_", "frame", bodyName);
                        geometrySetNode.insertNodeAfter(geometrySetNode.element_end(), meshNode);
                    }
                }
                else {
                    SimTK::Xml::element_iterator geomSetIter = visObjElement.element_begin("GeometrySet");
                    if (geomSetIter != visObjElement.element_end()) {
                        convertDisplayGeometryToGeometryXML(aNode, outerScaleFactors, outerTransform, *geomSetIter);
                        geomSetIter->setElementTag("geometry");
                    }
                }
                // Regardless add FrameGeometry node for the display of the PhysicalFrame
                std::string physFrameName = aNode.getRequiredAttribute("name").getValue();
                SimTK::Xml::Element physFrameNode("FrameGeometry");
                physFrameNode.setAttributeValue("name", physFrameName + "_frame_geometry");
                XMLDocument::addConnector(physFrameNode, "Connector_Frame_", "frame", physFrameName);
                SimTK::Xml::Element appearanceNode("Appearance");
                SimTK::Xml::Element surface_properties{"SurfaceProperties"};
                surface_properties.setAttributeValue("name", 
                                                     "surface_properties");
                SimTK::Xml::Element frameRepresentation("representation");
                frameRepresentation.setValue("0");
                surface_properties.insertNodeAfter(surface_properties.element_end(), frameRepresentation);
                appearanceNode.insertNodeAfter(appearanceNode.element_end(), surface_properties);
                physFrameNode.insertNodeAfter(physFrameNode.element_end(), appearanceNode);
                SimTK::Xml::element_iterator geomSetIter = aNode.element_begin("geometry");
                if (geomSetIter != aNode.element_end()) {
                    geomSetIter->insertNodeAfter(geomSetIter->node_end(), physFrameNode);
                }
            }
        }
    }
    Super::updateFromXMLNode(aNode, versionNumber);
}

void PhysicalFrame::convertDisplayGeometryToGeometryXML(SimTK::Xml::Element& bodyNode,
    const SimTK::Vec3& outerScaleFactors, const SimTK::Vec6& outerTransform,
    SimTK::Xml::Element& geomSetElement) const
{
    std::string bodyName = bodyNode.getRequiredAttribute("name").getValue();

    SimTK::Xml::element_iterator objectsIter = geomSetElement.element_begin("objects");

    if (objectsIter != geomSetElement.element_end()) {
        SimTK::Xml::Element geometrySetNode("geometry");
        bodyNode.insertNodeAfter(bodyNode.element_end(), geometrySetNode);

        SimTK::Xml::element_iterator displayGeomIter = objectsIter->element_begin("DisplayGeometry");
        int counter = 1;
        while (displayGeomIter != objectsIter->element_end()) {
            // Create a <Mesh> Element and populate it
            SimTK::Xml::Element meshNode("Mesh");
            std::string geomName = bodyName + "_geom_" + to_string(counter);
            meshNode.setAttributeValue("name", geomName);
            // geometry_file
            std::string geomFile = "";
            SimTK::Xml::element_iterator geomFileIter = displayGeomIter->element_begin("geometry_file");
            if (geomFileIter != displayGeomIter->element_end()) {
                geomFile = geomFileIter->getValueAs<SimTK::String>();
            }
            // transform
            SimTK::Vec6 localXform(0.);
            SimTK::Xml::element_iterator localXformIter = displayGeomIter->element_begin("transform");
            if (localXformIter != displayGeomIter->element_end()) {
                localXform = localXformIter->getValueAs<SimTK::Vec6>();
            }
            if (localXform.norm() > SimTK::Eps) {
                // Create a Frame
                std::string frameName = bodyName + "_geom_frame_" + to_string(counter);
                SimTK::Xml::Element frameNode("frame_name", frameName);
                meshNode.insertNodeAfter(meshNode.element_end(), frameNode);

                SimTK::Xml::Element modelNode = bodyNode;
                do {
                    modelNode = modelNode.getParentElement();
                    SimTK::String edump;
                    modelNode.writeToString(edump);
                } while (modelNode.getElementTag() != "Model" && !modelNode.isTopLevelNode());

                SimTK::Xml::element_iterator frameSetIter = modelNode.element_begin("FrameSet");
                SimTK::Xml::element_iterator frameSetObjectsIter;
                if (frameSetIter != modelNode.element_end()) {
                    frameSetObjectsIter = frameSetIter->element_begin("objects");
                }
                else {
                    SimTK::Xml::Element frameSetNode("FrameSet");
                    modelNode.insertNodeAfter(modelNode.element_end(), frameSetNode);
                    SimTK::Xml::Element frameSetObjectsNode("objects");
                    frameSetNode.insertNodeAfter(frameSetNode.element_end(), frameSetObjectsNode);
                    frameSetObjectsIter = frameSetNode.element_begin("objects");
                }
                createFrameForXform(frameSetObjectsIter, frameName, localXform, bodyName);

                XMLDocument::addConnector(meshNode, "Connector_Frame_", "frame", frameName);
            }
            else
                XMLDocument::addConnector(meshNode, "Connector_Frame_", "frame", bodyName);
            // scale_factor
            SimTK::Vec3 localScale(1.);
            SimTK::Xml::element_iterator localScaleIter = displayGeomIter->element_begin("scale_factors");
            if (localScaleIter != displayGeomIter->element_end()) {
                localScale = localScaleIter->getValueAs<SimTK::Vec3>();
            }
            // Now compose scale factors and transforms and create new node to insert into bodyNode
            SimTK::Xml::Element meshFileNode("mesh_file", geomFile);
            std::stringstream localScaleStr;
            localScaleStr << localScale[0] * outerScaleFactors[0] << " " << localScale[1] * outerScaleFactors[1]
                << " " << localScale[2] * outerScaleFactors[2];
            SimTK::Xml::Element scaleFactorsNode("scale_factors", localScaleStr.str());
            meshNode.insertNodeAfter(meshNode.element_end(), scaleFactorsNode);
            meshNode.insertNodeAfter(meshNode.element_end(), meshFileNode);
            SimTK::Xml::Element appearanceNode("Appearance");
            // Move color and opacity under Appearance
            SimTK::Xml::element_iterator colorIter = displayGeomIter->element_begin("color");
            if (colorIter != displayGeomIter->element_end()) {
                appearanceNode.insertNodeAfter(appearanceNode.element_end(), displayGeomIter->removeNode(colorIter));
            }
            SimTK::Xml::element_iterator opacityIter = displayGeomIter->element_begin("opacity");
            if (opacityIter != displayGeomIter->element_end()) {
                appearanceNode.insertNodeAfter(appearanceNode.element_end(), displayGeomIter->removeNode(opacityIter));
            }
            SimTK::Xml::element_iterator reprIter = displayGeomIter->element_begin("display_preference");
            if (reprIter != displayGeomIter->element_end()) {
                reprIter->setElementTag("representation");
                if (reprIter->getValue() == "4") {
                    // Enum changed to go 0-3 instead of 0-4
                    SimTK::String rep = "3";
                    reprIter->setValue(rep);
                }
                SimTK::Xml::Element surface_properties("SurfaceProperties");
                surface_properties.setAttributeValue("name", 
                                                     "surface_properties");
                surface_properties.insertNodeAfter(surface_properties.element_end(), displayGeomIter->removeNode(reprIter));
                appearanceNode.insertNodeAfter(appearanceNode.element_end(),
                                               surface_properties);
            }
            meshNode.insertNodeAfter(meshNode.element_end(), appearanceNode);
            // Insert Mesh into parent
            geometrySetNode.insertNodeAfter(geometrySetNode.element_end(), meshNode);
            displayGeomIter++;
            counter++;
        }
    }
}

// This private method creates a frame in the owner model document with passed in name and content relative to bodyName
void PhysicalFrame::createFrameForXform(const SimTK::Xml::element_iterator& frameSetIter, const std::string& frameName, const SimTK::Vec6& localXform, const std::string& bodyName) const
{
    SimTK::Xml::Element frameNode("PhysicalOffsetFrame");
    frameNode.setAttributeValue("name", frameName);
    stringstream ss;
    ss << localXform[3] << " " << localXform[4] << " " << localXform[5];
    SimTK::Xml::Element translationNode("translation", ss.str());
    ss.clear();
    ss << localXform[0] << " " << localXform[1] << " " << localXform[2];
    SimTK::Xml::Element orientationNode("rotation", ss.str());
    frameNode.insertNodeAfter(frameNode.element_end(), translationNode);
    frameNode.insertNodeAfter(frameNode.element_end(), orientationNode);
    frameSetIter->insertNodeAfter(frameSetIter->element_end(), frameNode);
    XMLDocument::addConnector(frameNode, "Connector_PhysicalFrame_", "parent", bodyName);

}
