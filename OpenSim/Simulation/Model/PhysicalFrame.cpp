/* --------------------------------------------------------------------------*
*                         OpenSim:  PhysicalFrame.cpp                        *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2017 Stanford University and the Authors                *
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
}

void PhysicalFrame::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();
}

const SimTK::MobilizedBody& PhysicalFrame::getMobilizedBody() const
{
    return getModel().getMatterSubsystem().getMobilizedBody(_mbIndex);
}

SimTK::MobilizedBody& PhysicalFrame::updMobilizedBody() 
{
    return updModel().updMatterSubsystem().updMobilizedBody(_mbIndex);
}

void PhysicalFrame::setMobilizedBodyIndex(const SimTK::MobilizedBodyIndex& mbix) const
{
    OPENSIM_THROW_IF_FRMOBJ(!mbix.isValid(), Exception,
        "Attempting to assign an invalid SimTK::MobilizedBodyIndex");
    const_cast<Self*>(this)->_mbIndex = mbix;
}

/*
* Implementation of Frame interface by PhysicalFrame.
* 
*/
SimTK::Transform PhysicalFrame::
    calcTransformInGround(const SimTK::State& s) const
{
    // return X_GF = X_GB * X_BF;
    return getMobilizedBody().getBodyTransform(s);
}

SimTK::SpatialVec PhysicalFrame::
calcVelocityInGround(const SimTK::State& state) const
{
    return getMobilizedBody().getBodyVelocity(state);
}

SimTK::SpatialVec PhysicalFrame::
calcAccelerationInGround(const SimTK::State& state) const
{
    return getMobilizedBody().getBodyAcceleration(state);
}


SimTK::Transform PhysicalFrame::extendFindTransformInBaseFrame() const
{
    return SimTK::Transform();
}

void PhysicalFrame::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);

    for (int i = 0; i < get_WrapObjectSet().getSize(); i++)
        get_WrapObjectSet()[i].setFrame(*this);
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
                    SimTK::Xml::Element geometrySetNode("attached_geometry");
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
                        // The transform is relative to the owning component
                        // (this PhysicalFrame), so the connectee name just
                        // points up to this component ("..").
                        XMLDocument::addConnector(meshNode, "Connector_Frame_",
                                "frame", "..");
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
            }
        }
    }
    Super::updateFromXMLNode(aNode, versionNumber);
}

void PhysicalFrame::convertDisplayGeometryToGeometryXML(SimTK::Xml::Element& bodyNode,
    const SimTK::Vec3& outerScaleFactors, const SimTK::Vec6& outerTransformVec6,
    SimTK::Xml::Element& geomSetElement)
{
    std::string bodyName = bodyNode.getRequiredAttribute("name").getValue();

    SimTK::Xml::element_iterator objectsIter = geomSetElement.element_begin("objects");

    if (objectsIter != geomSetElement.element_end()) {
        SimTK::Xml::Element geometrySetNode("attached_geometry");
        bodyNode.insertNodeAfter(bodyNode.element_end(), geometrySetNode);

        SimTK::Xml::element_iterator displayGeomIter = objectsIter->element_begin("DisplayGeometry");
        int counter = 1;
        while (displayGeomIter != objectsIter->element_end()) {
            // Create a <Mesh> Element and populate it
            bool attachToThisFrame = true;
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
            SimTK::Vec6 innerXformVec6(0.);
            SimTK::Xml::element_iterator innerXformIter = displayGeomIter->element_begin("transform");
            if (innerXformIter != displayGeomIter->element_end()) {
                innerXformVec6 = innerXformIter->getValueAs<SimTK::Vec6>();
            }
            // Old geometry/visibleObject could specify 2 transforms one per DisplayGeometry 
            // and one for the whole visibleObject.
            // We'll attach to the frame only if both transforms are identity
            attachToThisFrame = (innerXformVec6.norm() < SimTK::Eps)
                && (outerTransformVec6.norm() < SimTK::Eps);
            // In practice no models were found where both innerTransform and outerTransform were
            // non trivial or scale factors were specified in both
            if (!attachToThisFrame) {
                // Create a Vec6 by composing inner and outer transforms
                SimTK::Transform outerTransform(outerTransformVec6.getSubVec<3>(3));
                outerTransform.updR().setRotationToBodyFixedXYZ(outerTransformVec6.getSubVec<3>(0));
                SimTK::Transform innerTransform(innerXformVec6.getSubVec<3>(3));
                innerTransform.updR().setRotationToBodyFixedXYZ(innerXformVec6.getSubVec<3>(0));
                SimTK::Transform composed = outerTransform.compose(innerTransform);
                SimTK::Vec6 composedXformVec6(0.);
                composedXformVec6.updSubVec<3>(3) = composed.p();
                composedXformVec6.updSubVec<3>(0) = composed.R().convertRotationToBodyFixedXYZ();
                // Create a new frame and attach geometry to it
                std::string frameName = bodyName + "_geom_frame_" + to_string(counter);
                SimTK::Xml::Element frameNode("frame_name", frameName);
                meshNode.insertNodeAfter(meshNode.element_end(), frameNode);

                SimTK::Xml::element_iterator componentsNode =
                    bodyNode.element_begin("components");

                if (componentsNode == bodyNode.element_end()) {
                    SimTK::Xml::Element componentsElement("components");
                    bodyNode.insertNodeBefore(bodyNode.element_end(), componentsElement);
                    componentsNode = bodyNode.element_begin("components");
                }

                // Create Frame from composed transform and insert in
                // components list.
                // The transform is relative to the owning component (this
                // PhysicalFrame), so the connectee name just points up to this
                // component ("..").
                createFrameForXform(componentsNode, frameName,
                        composedXformVec6, "..");

                // For Geometry under 'attached_geometry', the 'frame' is
                // always the owning component.
                XMLDocument::addConnector(meshNode, "Connector_Frame_", "frame", "..");
                SimTK::Xml::element_iterator parentFrame = componentsNode->element_begin("PhysicalOffsetFrame");
                while (parentFrame->getRequiredAttributeValue("name") != frameName)
                    parentFrame++;

                // if parentFrame has no "attached_geometry" child, add one else find it
                SimTK::Xml::element_iterator frameAttachedGeometry = parentFrame->element_begin("attached_geometry");
                if (frameAttachedGeometry == parentFrame->element_end()) {
                    SimTK::Xml::Element attachedGeomNode("attached_geometry");
                    parentFrame->insertNodeAfter(parentFrame->node_end(), attachedGeomNode);
                    frameAttachedGeometry = parentFrame->element_begin("attached_geometry");
                }
                frameAttachedGeometry->insertNodeAfter(frameAttachedGeometry->element_end(), meshNode);

            }
            else
                // For Geometry under 'attached_geometry', the 'frame' is
                // always the owning component.
                XMLDocument::addConnector(meshNode, "Connector_Frame_",
                        "frame", "..");
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
                appearanceNode.insertNodeAfter(appearanceNode.element_end(), displayGeomIter->removeNode(reprIter));
            }
            meshNode.insertNodeAfter(meshNode.element_end(), appearanceNode);
            // Insert Mesh into parent
            if (attachToThisFrame)
                geometrySetNode.insertNodeAfter(geometrySetNode.element_end(), meshNode);
                        
            displayGeomIter++;
            counter++;
        }
    }
}

void PhysicalFrame::createFrameForXform(
        const SimTK::Xml::element_iterator& ownerIter,
        const std::string& frameName, const SimTK::Vec6& localXform,
        const std::string& parentConnecteeName)
{
    SimTK::Xml::Element frameNode("PhysicalOffsetFrame");
    frameNode.setAttributeValue("name", frameName);
    stringstream ssT;
    ssT << localXform[3] << " " << localXform[4] << " " << localXform[5];
    SimTK::Xml::Element translationNode("translation", ssT.str());
    stringstream ssO;
    ssO << localXform[0] << " " << localXform[1] << " " << localXform[2];
    SimTK::Xml::Element orientationNode("orientation", ssO.str());
    frameNode.insertNodeAfter(frameNode.element_end(), translationNode);
    frameNode.insertNodeAfter(frameNode.element_end(), orientationNode);
    ownerIter->insertNodeAfter(ownerIter->element_end(), frameNode);
    XMLDocument::addConnector(frameNode, "Connector_PhysicalFrame_", "parent",
            parentConnecteeName);
}
