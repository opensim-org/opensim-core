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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>

namespace OpenSim {

class Body;
class Model;

using SimTK::DecorativeGeometry;
/**
SurfaceAppearance class holds the display attributes  of a piece of Geometry 
displayed in the OpenSim visualizer or GUI. The attributes in this 
class are specific to geometry that have surfaces so these surfaces can be 
textured, or rendered using  a variety of shading/lighting models etc. Attributes 
common to all possible  objects that can be rendered (e.g. color, opacity, 
visibility) are not included here but provided by the Appearance class instead.
*/

class OSIMSIMULATION_API SurfaceAppearance : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(SurfaceAppearance, Object);
public:
    //==========================================================================
    // PROPERTIES
    //==========================================================================
    OpenSim_DECLARE_PROPERTY(representation, int,
        "The representation (1:Points, 2:Wire 3:Shaded) used to display the object. ");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(texture_file, std::string,
        "Name of file containing texture. ");

    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    SurfaceAppearance() {
        constructProperty_representation(3); // Surface shaded by default
    }
    virtual ~SurfaceAppearance() {};

    bool hasTexture() {
        return getProperty_texture_file().size() > 0;
    }
    //=============================================================================
};  // END of class SurfaceAppearance

//=============================================================================
/** CurveAppearance class holds the display attributes  of a piece of Geometry
displayed in the OpenSim visualizer or GUI.The attributes in this
class are specific to curves or line drawings so that their thickness, line-style
etc. can be maintained. Attributes common to all possible geometries that can be 
rendered (e.g.color, opacity, visibility) are not included
here but provided by the Appearance class instead.
*/
//=============================================================================
class OSIMSIMULATION_API CurveAppearance : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(CurveAppearance, Object);
public:
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    OpenSim_DECLARE_PROPERTY(thickness, double,
        "The thickness of lines used to render a curve or a drawing. ");

    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    CurveAppearance() {
        constructProperties();
    }
    virtual ~CurveAppearance() {};

private:
    void constructProperties() {
        constructProperty_thickness(.05);
    }
    //=============================================================================
};  // END of class CurveAppearance

//=============================================================================
//=============================================================================
/**
 * A class that holds the display attributes (Appearance) of an object displayed 
 * in the OpenSim Visualizer.
 * 
 * Appearance contains properties that apply to all geometry.
 * Geometry that have a surface so that it can be textured will utilize
 * surface_ppearance, while schematic line drawings (e.g. Arrows, Frames) 
 * can utilize curve_ppearance which offers thickness. 
 *
 * TODO: Add Resolution to Appearance from DecorativeGeometry and 
 * utilize  for Arrow, Line, and Frame (unused for now)
 *
 * @author Ayman Habib
 * @version 1.0
 */
class OSIMSIMULATION_API Appearance : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(Appearance, Object);
public:
    //==========================================================================
    // PROPERTIES
    //==========================================================================
    OpenSim_DECLARE_PROPERTY(visibile, bool, 
        "Flag indicating whether the associated Geometry is visible or hidden.")
    OpenSim_DECLARE_PROPERTY(opacity, double,
            "The opacity (0-1) used to display the object. ");
    OpenSim_DECLARE_PROPERTY(color, SimTK::Vec3,
        "The color (rgb) used to display the object. ");

    OpenSim_DECLARE_PROPERTY(surface_appearance, SurfaceAppearance,
        "Visuals applied to surfaces associated with this Appearance. ");
    OpenSim_DECLARE_PROPERTY(curve_appearance, CurveAppearance,
        "Visuals applied to curves or line drawings associated with this Appearance. ");


    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    Appearance() {
        constructProperties();
    }
    virtual ~Appearance() {};

    DecorativeGeometry::Representation get_representation() const { return (DecorativeGeometry::Representation)get_surface_appearance().get_representation(); }

    void set_representation(DecorativeGeometry::Representation& rep) { upd_surface_appearance().set_representation(rep); }

private:
    void constructProperties() {
        constructProperty_visibile(true);
        constructProperty_opacity(1.0);
        constructProperty_color(SimTK::Vec3(1.0)); // White by default, shows as a shade of gray
        constructProperty_surface_appearance(SurfaceAppearance());
        constructProperty_curve_appearance(CurveAppearance());
    }
    //==========================================================================
};  // END of class Appearance

} // end of namespace OpenSim

#endif // OPENSIM_APPEARANCE_H_


