#ifndef OPENSIM_APPEARANCE_H_
#define OPENSIM_APPEARANCE_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Appearance.h                         *
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


// INCLUDE
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/XMLDocument.h>
#include <SimTKcommon/internal/DecorativeGeometry.h>

namespace OpenSim {

class Body;
class Model;

/**
VisualRepresentation is the OpenSim name used across the OpenSim API, it is an 
that describes in what form is Geometry displayed:  
DrawPoints, DrawWireframe, DrawSurface are supported.
*/
#ifndef SWIG
using VisualRepresentation = SimTK::DecorativeGeometry::Representation;
#else
typedef VisualRepresentation SimTK::DecorativeGeometry::Representation;
#endif

/**
SurfaceProperties class holds the appearance properties of a piece of Geometry 
displayed in the OpenSim visualizer or GUI as a surface. The properties in this 
class are specific to geometry that have surfaces so that these surfaces can be 
textured, or rendered using  a variety of shading models. 
*/

class OSIMSIMULATION_API SurfaceProperties : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(SurfaceProperties, Object);
public:
    //==========================================================================
    // PROPERTIES
    //==========================================================================
    OpenSim_DECLARE_PROPERTY(representation, int,
     "The representation (1:Points, 2:Wire, 3:Shaded) used to display the object.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(texture, std::string,
        "Name of texture e.g. metal, bone. This is a hint to the GUI/Visualizer, implementation is Visualization dependent.");

    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    SurfaceProperties() {
        // Surface shaded by default
        constructProperty_representation(VisualRepresentation::DrawSurface);
        constructProperty_texture();
    }
    virtual ~SurfaceProperties() {};

    bool hasTexture() {
        return !getProperty_texture().empty();
    }
    //========================================================================
};  // END of class SurfaceProperties

//=============================================================================
//=============================================================================
/**
 * A class that holds the Appearance properties of Geometry displayed 
 * in the OpenSim Visualizer. It affects how Geometry is displayed.
 * 
 * Appearance contains properties that apply to all geometry.
 * Geometry that have a surface so that it can be textured will utilize
 * SurfaceProperties, while schematic line drawings (e.g. Arrows, Frames) 
 * can utilize CurveProperties which offers thickness. 
 *
 * @author Ayman Habib
 */
class OSIMSIMULATION_API Appearance : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(Appearance, Object);
public:

    //==========================================================================
    // PROPERTIES
    //==========================================================================
    OpenSim_DECLARE_PROPERTY(visible, bool, 
        "Flag indicating whether the associated Geometry is visible or hidden.")
    OpenSim_DECLARE_PROPERTY(opacity, double,
        "The opacity used to display the geometry between 0:transparent, 1:opaque.");
    OpenSim_DECLARE_PROPERTY(color, SimTK::Vec3,
        "The color, (red, green, blue), [0, 1], used to display the geometry. ");

    OpenSim_DECLARE_UNNAMED_PROPERTY(SurfaceProperties,
        "Visuals applied to surfaces associated with this Appearance.");


    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    Appearance() {
        constructProperties();
    }
    virtual ~Appearance() {};

    OpenSim::VisualRepresentation get_representation() const { 
        return (VisualRepresentation)this->get_SurfaceProperties().get_representation(); }

    void set_representation(const OpenSim::VisualRepresentation& rep) { 
        upd_SurfaceProperties().set_representation(rep); }

protected:
    /** Updating XML formating to latest revision */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) override {
        int documentVersion = versionNumber;
        if (documentVersion < XMLDocument::getLatestVersion()) {
            if (documentVersion < 30505) {
                // if representation = 0 then  visible = false else  visible = true
                SimTK::Xml::element_iterator iIter = 
                    aNode.element_begin("representation");
                if (iIter != aNode.element_end()) {
                    int oldRep = iIter->getValueAs<int>();
                    if (oldRep == 0) {
                        SimTK::Xml::Element visNode("visible", "false");
                        aNode.insertNodeAfter(aNode.element_end(), visNode);
                    }

                }
            }
        }
        Super::updateFromXMLNode(aNode, versionNumber);
    }

private:
    void constructProperties() {
        constructProperty_visible(true);
        constructProperty_opacity(1.0);
        // White by default, shows as a shade of gray
        constructProperty_color(SimTK::White); 
        constructProperty_SurfaceProperties(SurfaceProperties());
    }
    //=========================================================================
};  // END of class Appearance

} // end of namespace OpenSim

#endif // OPENSIM_APPEARANCE_H_


