/* -------------------------------------------------------------------------- *
 *                       OpenSim:  SpatialTransform.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
#include "SpatialTransform.h"
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/MultiplierFunction.h>
#include <OpenSim/Common/LinearFunction.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================

//_____________________________________________________________________________
/*
 * Default constructor of a SpatialTransform.
 */
SpatialTransform::SpatialTransform()
{
    setNull();
    constructProperties();
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this SpatialTransform to their null values.
 */
void SpatialTransform::setNull()
{
    setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Initialize properties.
 */
void SpatialTransform::constructProperties()
{
    constructProperty_rotation1(TransformAxis(Array<string>(), Vec3(1,0,0)));
    constructProperty_rotation2(TransformAxis(Array<string>(), Vec3(0,1,0)));
    constructProperty_rotation3(TransformAxis(Array<string>(), Vec3(0,0,1)));
    constructProperty_translation1(TransformAxis(Array<string>(), Vec3(1,0,0)));
    constructProperty_translation2(TransformAxis(Array<string>(), Vec3(0,1,0)));
    constructProperty_translation3(TransformAxis(Array<string>(), Vec3(0,0,1)));
}


const TransformAxis& SpatialTransform::getTransformAxis(int whichAxis) const
{
     switch(whichAxis){
        case 0: return get_rotation1();
        case 1: return get_rotation2();
        case 2: return get_rotation3();
        case 3: return get_translation1();
        case 4: return get_translation2();
        case 5: return get_translation3();
        default:
            throw Exception("SpatialTransform: Attempting to access beyond 6 axes.");
     }
}

TransformAxis& SpatialTransform::updTransformAxis(int whichAxis)
{
     switch(whichAxis){
        case 0: return upd_rotation1();
        case 1: return upd_rotation2();
        case 2: return upd_rotation3();
        case 3: return upd_translation1();
        case 4: return upd_translation2();
        case 5: return upd_translation3();
        default:
            throw Exception("SpatialTransform: Attempting to access beyond 6 axes.");
     }
}

// SETUP
void SpatialTransform::connectToJoint(CustomJoint& owningJoint)
{
    // define default function for TransformAxes that have none specified
    for(int i=0; i < NumTransformAxes; ++i) {
        TransformAxis& transform = updTransformAxis(i);

        // check if it has a function
        if(!transform.hasFunction()){
            // does it have a coordinate?
            if(transform.getCoordinateNames().size() == 1)
                transform.setFunction(new LinearFunction());
            else if(transform.getCoordinateNames().size() > 1)
                throw Exception("TransformAxis: an appropriate multi-coordinate function was not supplied");
            else
                transform.setFunction(new Constant());
        }

        // Ask the transform axis to connect itself to the joint.
        transform.connectToJoint(*((Joint*)(&owningJoint)));
    }
}

// Spatial Transform specific methods
OpenSim::Array<string> SpatialTransform::getCoordinateNames() const
{
    OpenSim::Array<string> coordinateNames;

    for(int i=0; i < NumTransformAxes; i++){
        const TransformAxis& transform = getTransformAxis(i);
        for(int j = 0; j < transform.getCoordinateNames().size(); j++){
            string name = transform.getCoordinateNames()[j];
            if(coordinateNames.findIndex(name) < 0)
                coordinateNames.append(name);
        }
    }
    return coordinateNames;
}

std::vector<std::vector<int> > SpatialTransform::getCoordinateIndices() const
{
    std::vector<std::vector<int> > coordIndices(6);
    Array<string> coordinateNames = getCoordinateNames();

    for(int i=0; i < NumTransformAxes; i++){
        const TransformAxis& transform = getTransformAxis(i);
        // Get the number of coordinates that dictate motion along this axis
        int ncoords = transform.getCoordinateNames().size();
        std::vector<int> findex(ncoords);
        for(int j=0; j< ncoords; j++){
            int ind = coordinateNames.findIndex(transform.getCoordinateNames()[j]);
            if (ind > -1)
                findex[j] = ind;
        }
        coordIndices[i] = findex;   
    }
    
    return coordIndices;
}
std::vector<const SimTK::Function*> SpatialTransform::getFunctions() const
{
    std::vector<const SimTK::Function*> functions(NumTransformAxes);
    for(int i=0; i < NumTransformAxes; i++){
        functions[i] = getTransformAxis(i).getFunction().createSimTKFunction();
    }
    return functions;
}
std::vector<SimTK::Vec3> SpatialTransform::getAxes() const
{
    std::vector<SimTK::Vec3> axes(NumTransformAxes);
    for(int i=0; i<NumTransformAxes; i++){
        axes[i] = getTransformAxis(i).getAxis();
    }

    return axes;
}


void SpatialTransform::scale(const SimTK::Vec3 scaleFactors)
{
    // Scale the spatial transform functions of translations only
    for (int i = 3; i < NumTransformAxes; i++) {
        TransformAxis& transform = updTransformAxis(i);
        if (transform.hasFunction()) {
            Function& function = transform.updFunction();
            // If the function is a linear function with coefficients of 1.0 and 0.0, do
            // not scale it because this transform axis represents a degree of freedom.
            LinearFunction* lf = dynamic_cast<LinearFunction*>(&function);
            if (lf) {
                const Array<double> coefficients = lf->getCoefficients();
                if (coefficients[0] == 1.0 && coefficients[1] == 0.0)
                    continue;
            }
            SimTK::Vec3 axis;
            transform.getAxis(axis);
            double scaleFactor = ~axis * scaleFactors;
            // If the function is already a MultiplierFunction, just update its scale factor.
            // Otherwise, make a MultiplierFunction from it and make the transform axis use
            // the new MultiplierFunction.
            MultiplierFunction* mf = dynamic_cast<MultiplierFunction*>(&function);
            if (mf) {
                mf->setScale(mf->getScale() * scaleFactor);
            } 
            else {
                mf = new MultiplierFunction();
                mf->setScale(scaleFactor);
                // Make a copy of the original function and delete the original
                // (so its node will be removed from the XML document).
                mf->setFunction(function.clone());
                transform.setFunction(mf);
            }
        }
    }
}
/**
 * constructIndependentAxes checks if the TransformAxis at indices startIndex, 
 * startIndex+1, startIndex+2 
 * are independent and fixes them otherwise. It assumes that the first nAxes are ok
 */
void SpatialTransform::constructIndependentAxes(int nAxes, int startIndex)
{
    if (nAxes == 3 || nAxes==0) return;     // Nothing to do
    Vec3 v1 = getTransformAxis(0+startIndex).getAxis();
    Vec3 v2 = getTransformAxis(1+startIndex).getAxis();
    Vec3 v3 = getTransformAxis(2+startIndex).getAxis();
    if (nAxes ==2){ // Easy, make third axis the cross of the first 2.
        SimTK::Vec3 cross= (v1 % v2);
        cross.normalize();
        updTransformAxis(2+startIndex).setAxis(cross);
    }
    else {  // only v1 was specified, check if v2 is collinear if so, exchange v2, v3
        if (fabs(fabs(~v1 * v2) -1) < 1e-4){
            updTransformAxis(1+startIndex).setAxis(v3);
        }
        v2 = getTransformAxis(1+startIndex).getAxis();
        SimTK::Vec3 cross= (v2 % v1);
        cross.normalize();
        updTransformAxis(2+startIndex).setAxis(cross);  
    }
}
