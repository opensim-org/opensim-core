#ifndef OPENSIM_PROPERTY_HELPER_H_
#define OPENSIM_PROPERTY_HELPER_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PropertyHelper.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyTransform.h>
#include <OpenSim/Common/Array.h>

namespace OpenSim {
//==============================================================================
//                                 PropertyHelper
//==============================================================================
/**
This class allows access to property values using template-free
 methods. Note that this will work regardless of whether the given
 AbstractProperty is the deprecated kind or the new one.

 An AbstractProperty represents a (name, list-of-values) pair, possibly
 with restrictions on the minimum and maximum list length. Basic container
 methods size(), resize(), clear(), and empty() are available; use resize()
 before assigning a value to an indexed element.

 For properties that contain objects, you can obtain the values directly
 from the base class via non-templatized methods.
 **/
class PropertyHelper {
public:
    //=================Boolean Properties==================
    // Recover boolean value from an AbstractProperty that was assumed to contain a boolean
    // Will throw exception if the assumption was wrong/invalid. Use index only if the
    // property contains an array of booleans.
    static bool getValueBool(const AbstractProperty& p, int index=-1) 
    {   return p.getValue<bool>(index); }
    // Set boolean value in an AbstractProperty that was assumed to hold a boolean
    // Will throw exception if the assumption was wrong/invalid. Use index only if the
    // property contains an array of booleans.
    static void setValueBool(bool v, AbstractProperty& p, int index=-1) 
    {   p.updValue<bool>(index) = v; }
    // Append a new boolean value to an AbstractProperty that was assumed to hold a variable size
    // array of booleans. Will throw exception if the assumption was wrong/invalid. 
    static void appendValueBool(bool v, AbstractProperty& p) 
    {   p.appendValue<bool>(v); }
    //=================Int Properties, see Boolean Properties for details ==================
    static int getValueInt(const AbstractProperty& p, int index=-1) 
    {   return p.getValue<int>(index); }
    static void setValueInt(int v, AbstractProperty& p, int index=-1) 
    {   p.updValue<int>(index) = v; }
    static void appendValueInt(int v, AbstractProperty& p) 
    {   p.appendValue<int>(v); }
    //=================Double Properties, see Boolean Properties for details ==================
    static double getValueDouble(const AbstractProperty& p, int index=-1) 
    {   return p.getValue<double>(index); }
    static void setValueDouble(double v, AbstractProperty& p, int index=-1) 
    {   p.updValue<double>(index) = v; }
    static void appendValueDouble(double v, AbstractProperty& p) 
    {   p.appendValue<double>(v); }
    //=================String Properties, see Boolean Properties for details ==================
    static std::string getValueString(const AbstractProperty& p, int index=-1) 
    {   return p.getValue<std::string>(index); }
    static void setValueString(const std::string& v, 
                               AbstractProperty& p, int index=-1) 
    {   p.updValue<std::string>(index) = v; }
    static void appendValueString(const std::string& v, AbstractProperty& p) 
    {   p.appendValue<std::string>(v); }
    //=================Transform Properties, treated as six Doubles ==================
    static double getValueTransform(const AbstractProperty& p, int index) 
    {   
        const PropertyTransform& pd = dynamic_cast<const PropertyTransform&>(p);
        double array6[] = {0., 0., 0., 0., 0., 0.};
        pd.getRotationsAndTranslationsAsArray6(array6);
        return array6[index]; 
    }
    static void setValueTransform(double v, AbstractProperty& p, int index) 
    {   
        PropertyTransform& pd = dynamic_cast<PropertyTransform&>(p);
        double array6[] = {0., 0., 0., 0., 0., 0.};
        pd.getRotationsAndTranslationsAsArray6(array6);
        array6[index] = v; 
        pd.setValue(6, array6);
    }
    //=================Vec3 Properties, treated as three Doubles ==================
    static double getValueVec3(const AbstractProperty& p, int index)
    {
        const Property<SimTK::Vec3>& pd = dynamic_cast<const Property<SimTK::Vec3>&>(p);
        const SimTK::Vec3& vec3 = pd.getValue();
        return vec3[index];
    }
    static void setValueVec3(double v, AbstractProperty& p, int index)
    {
        Property<SimTK::Vec3>& pd = dynamic_cast<Property<SimTK::Vec3>&>(p);
        pd.updValue()[index] = v;
    }
    static double getValueVec6(const AbstractProperty& p, int index)
    {
        const Property<SimTK::Vec6>& pd = dynamic_cast<const Property<SimTK::Vec6>&>(p);
        const SimTK::Vec6& vec6 = pd.getValue();
        return vec6[index];
    }
    static void setValueVec6(double v, AbstractProperty& p, int index)
    {
        Property<SimTK::Vec6>& pd = dynamic_cast<Property<SimTK::Vec6>&>(p);
        pd.updValue()[index] = v;
    }
    // ================ String arrays ===================================================
    static OpenSim::Array<std::string> getValueStringArray(const AbstractProperty& p)
    {
        OpenSim::Array<std::string> val = OpenSim::Array<std::string>();
        for (int i=0; i< p.size(); i++)
            val.append(p.getValue<std::string>(i));
        return val;
    }
    static void setValueStringArray(AbstractProperty& p,  OpenSim::Array<std::string>& aStringArray)
    {
        p.clear();
        for (int i=0; i< aStringArray.getSize(); i++)
            try {
                p.appendValue<std::string>(aStringArray.get(i));
            } catch (OpenSim::Exception e) {
                OpenSim::Exception ex("ERROR- Invalid input (invalid character/spaces in input string)");
                throw ex;
            }
    }

    static void removeItem(AbstractProperty& p, int index)
    {
        if (p.size()>index){
            AbstractProperty* cloneP = p.clone();
            p.clear();
            for(int i=0; i<cloneP->size();i++){
                if (i!= index){
                if (p.getTypeName()=="string")
                    p.appendValue(cloneP->getValue<std::string>(i));
                else if (p.getTypeName()=="int")
                    p.appendValue(cloneP->getValue<int>(i));
                else if (p.getTypeName()=="double")
                    p.appendValue(cloneP->getValue<double>(i));
                else if (p.getTypeName()=="bool")
                    p.appendValue(cloneP->getValue<bool>(i));
                }
            }
        }
    }

};

} // namespace OpenSim

#endif // OPENSIM_PROPERTY_HELPER_H_

