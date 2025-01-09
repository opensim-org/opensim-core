/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ModelComponent.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Michael Sherman                                      *
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

// INCLUDES
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "OpenSim/Simulation/Model/Model.h"

using namespace SimTK;

namespace OpenSim {

//==============================================================================
//                             MODEL COMPONENT
//==============================================================================
ModelComponent::ModelComponent() : Component() {}

ModelComponent::ModelComponent(const std::string& fileName, bool updFromXMLNode)
:   Component(fileName, updFromXMLNode)
{}

ModelComponent::ModelComponent(SimTK::Xml::Element& element) 
:   Component(element)
{}

const Model& ModelComponent::getModel() const
{
    if(!_model)
        throw Exception("ModelComponent::getModel(): component '" + getName() +
        "' of type " + getConcreteClassName() + " does not belong to a model. "
            "Have you called Model::initSystem()?"); 
    return *_model;
}

Model& ModelComponent::updModel()
{
    if(!_model)
        throw Exception("ModelComponent::updModel(): component '" + getName() +
        "' of type " + getConcreteClassName() + " does not belong to a model. "
            "Have you called Model::initSystem()?");
    return *_model;
}


void ModelComponent::extendFinalizeConnections(Component& root)
{
    Super::extendFinalizeConnections(root);
    Model* model = dynamic_cast<Model*>(&root);
    // Allow (model) component to include its own subcomponents
    // before calling the base method which automatically invokes
    // connect all the subcomponents.
    if (model)
        connectToModel(*model);
}

void ModelComponent::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();
}

// Base class implementation of virtual method.
void ModelComponent::connectToModel(Model& model)
{
    _model = &model;
    extendConnectToModel(model);
}

const SimTK::DefaultSystemSubsystem& ModelComponent::
getDefaultSubsystem() const
{   return getModel().getDefaultSubsystem(); }

const SimTK::DefaultSystemSubsystem& ModelComponent::
updDefaultSubsystem()
{   return updModel().updDefaultSubsystem(); }


void ModelComponent::updateFromXMLNode(SimTK::Xml::Element& aNode,
        int versionNumber) {

    if (versionNumber < XMLDocument::getLatestVersion()) {
        if (versionNumber < 30506) {
            // geometry list property removed. Everything that was in this list
            // should be moved to the components list property.
            SimTK::Xml::element_iterator geometry = aNode.element_begin("geometry");
            if (geometry != aNode.element_end()) {
                // We found a list property of geometry.
                SimTK::Xml::Element componentsNode;
                SimTK::Xml::element_iterator componentsIt = aNode.element_begin("components");
                if (componentsIt == aNode.element_end()) {
                    // This component does not yet have a list property of
                    // components, so we'll create one.
                    componentsNode = SimTK::Xml::Element("components");
                    aNode.insertNodeBefore(aNode.element_begin(), componentsNode);
                } else {
                    componentsNode = *componentsIt;
                }
                // Copy each node under <geometry> into <components>.
                for (auto geomIt = geometry->element_begin();
                        geomIt != geometry->element_end(); ++geomIt) {
                    componentsNode.appendNode(geomIt->clone());
                }
                // Now that we moved over the geometry, we can delete the
                // <geometry> element.
                aNode.eraseNode(geometry);
            }
        }
    }
    Super::updateFromXMLNode(aNode, versionNumber);
}

// Base class implementations of virtual methods for scaling.
void ModelComponent::preScale(const SimTK::State& s, const ScaleSet& scaleSet)
{   extendPreScale(s, scaleSet); }

void ModelComponent::scale(const SimTK::State& s, const ScaleSet& scaleSet)
{   extendScale(s, scaleSet); }

void ModelComponent::postScale(const SimTK::State& s, const ScaleSet& scaleSet)
{   extendPostScale(s, scaleSet); }

// (static) Returned by getScaleFactors() if scale factors not found.
const SimTK::Vec3 ModelComponent::InvalidScaleFactors = SimTK::Vec3(0);

const SimTK::Vec3& ModelComponent::
getScaleFactors(const ScaleSet& scaleSet, const Frame& frame) const
{
    const std::string& baseFrameName = frame.findBaseFrame().getName();

    int size = scaleSet.getSize();

    for (int i = 0; i < size; ++i) {
        if (scaleSet[i].getSegmentName() == baseFrameName) {
            if (scaleSet[i].getApply())
                return scaleSet[i].getScaleFactors();
        }
    }

    // No scale factors found for the base Body or not applicable.
    return InvalidScaleFactors;
}

} // end of namespace OpenSim
