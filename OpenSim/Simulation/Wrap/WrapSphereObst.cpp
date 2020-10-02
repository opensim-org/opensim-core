/* -------------------------------------------------------------------------- *
 *                        OpenSim:  WrapSphereObst.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Brian Garner, Peter Loan                                        *
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
#include "WrapSphereObst.h"
#include <OpenSim/Simulation/Wrap/WrapResult.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

static const char* wrapTypeName = "sphereObst";


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
* Default constructor.
*/
WrapSphereObst::WrapSphereObst()
{
    constructProperties();
}

//_____________________________________________________________________________
/**
* Destructor.
*/
WrapSphereObst::~WrapSphereObst()
{
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
* Connect properties to local pointers.
*/
void WrapSphereObst::constructProperties()
{
    constructProperty_radius(0.05);
    constructProperty_length(0.1);
}

//_____________________________________________________________________________
/**
* Perform some set up functions that happen after the
* object has been deserialized or copied.
*
* @param aModel point to OpenSim Model.
*/
void WrapSphereObst::extendFinalizeFromProperties()
{
    // Base class
    Super::extendFinalizeFromProperties();

    // maybe set a parent pointer, _body = aBody;
    OPENSIM_THROW_IF_FRMOBJ(
        get_radius() <= 0,
        InvalidPropertyValue,
        getProperty_radius().getName(),
        "Radius cannot be less than or equal zero");

    OPENSIM_THROW_IF_FRMOBJ(
        get_length() <= 0,
        InvalidPropertyValue,
        getProperty_length().getName(),
        "Length cannot be less than or equal zero");
    
/*  Sphere* sph = new Sphere(_radius);
    setGeometryQuadrants(sph);
*/
}

//_____________________________________________________________________________
/**
 * Get the name of the type of wrap object ("cylinderObst" in this case)
 *
 * @return A string representing the type of wrap object
 */
const char* WrapSphereObst::getWrapTypeName() const
{
    return wrapTypeName;
}

//_____________________________________________________________________________
/**
 * Get a string holding the dimensions definition that SIMM would
 * use to describe this object. This is a rather ugly convenience
 * function for outputting SIMM joint files.
 *
 * @return A string containing the dimensions of the wrap object
 */
string WrapSphereObst::getDimensionsString() const
{
    stringstream dimensions;
    dimensions << "radius " << get_radius() << "\nheight " << get_length();

    return dimensions.str();
}

//=============================================================================
// WRAPPING
//=============================================================================
//_____________________________________________________________________________
/**
 * Calculate the wrapping of one line segment over the sphere located at the obstacle origin.
 *
 * @param aPointP One end of the line segment, already expressed in obstacle frame
 * @param aPointS The other end of the line segment, already expressed in obstacle frame
 * @param aMuscleWrap An object holding the parameters for this line/cylinder pairing
 * @param aWrapResult The result of the wrapping (tangent points, etc.)
 * @param aFlag A flag for indicating errors, etc.
 * @return The status, as a WrapAction enum
 */
int WrapSphereObst::wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
                        const PathWrap& aMuscleWrap, WrapResult& aWrapResult, bool& aFlag) const
{
    SimTK::Vec3& aPointP = aPoint1;     double R=0.8*get_radius();
    SimTK::Vec3& aPointS = aPoint2;     double Qx,Qy, Tx,Ty;

    // Initialize return values
    aFlag = false;
    aWrapResult.wrap_path_length = 0.0;
    aWrapResult.wrap_pts.setSize(0);
    
    // Establish a local wrap coordinate system aligned with wrapping plane
    SimTK::Vec3 aXvec = aPointP;                aXvec = aXvec.normalize();  // X = P
    SimTK::Vec3 aZvec = aPointP % aPointS;
    if(aZvec.norm()<=1.e-7) {
        printf("WrapSphereObst: P and S are collinear with sphere center (no unique solution)\n");
        return insideRadius;
    }                                           aZvec = aZvec.normalize();  // Z = P x S
    SimTK::Vec3 aYvec = aZvec % aXvec;          aYvec = aYvec.normalize();  // Y = Z x X
    
    // Compute displacements of P and S from sphere center within wrap coordinate system
    double Px=aPointP.norm(), Py=0.0, /*Pz=0.0,*/               dP=Px*Px+Py*Py,  rootP=dP-R*R;
    double Sx=~aPointS*aXvec, Sy=~aPointS*aYvec, /*Sz=0.0,*/    dS=Sx*Sx+Sy*Sy,  rootS=dS-R*R;

    // Check P and S against sphere, and compute x and y components of wrap points Q and T
    if( rootP<0.0 || rootS<0.0 ) return insideRadius;   // One of P or S lies within the sphere
    dP=R/dP;    rootP=sqrt(rootP);  Qx=(R*Px-rootP*Py)*dP;  Qy=(R*Py+rootP*Px)*dP;
    dS=R/dS;    rootS=sqrt(rootS);  Tx=(R*Sx+rootS*Sy)*dS;  Ty=(R*Sy-rootS*Sx)*dS;
    // NOTE:  Qz and Tz are necessarily zero in the wrapping plane coordinate system since by definition they lie in the plane

    // Apply the 180-degree wrapping rule to see if contact is appropriate (i.e. wrap > 180 = no contact)
    if( R*(Qx*Ty-Qy*Tx) < 0.0 ) return noWrap;

    // Compute respective wrapping segment lengths
    //double PQ = sqrt( (Qx-Px)*(Qx-Px) + (Qy-Py)*(Qy-Py) );
    //double TS = sqrt( (Tx-Sx)*(Tx-Sx) + (Ty-Sy)*(Ty-Sy) );
    double QT = R*acos( 1.0 - 0.5*( (Qx-Tx)*(Qx-Tx) + (Qy-Ty)*(Qy-Ty) )/(R*R) );
    if(QT<0.0) QT=-QT;
    
    // Transform Q and T from wrap coordinate system back into obstacle coordinate system
    SimTK::Vec3 aPointQ = (aXvec*Qx) + (aYvec*Qy);
    SimTK::Vec3 aPointT = (aXvec*Tx) + (aYvec*Ty);

    // Register results and return
    aFlag = true;
    aWrapResult.wrap_path_length = QT;  // PQ + TS + QT;
    aWrapResult.r1=aPointQ;     //SimmPoint wppt1(aWrapResult.r1);  
    aWrapResult.wrap_pts.append(aWrapResult.r1);
    aWrapResult.r2=aPointT;     //SimmPoint wppt2(aWrapResult.r2);  
    aWrapResult.wrap_pts.append(aWrapResult.r2);

    return wrapped;
}




