/* -------------------------------------------------------------------------- *
 *                       OpenSim:  WrapCylinderObst.cpp                       *
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
#include "WrapCylinderObst.h"
#include <OpenSim/Simulation/Wrap/WrapResult.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

static const char* wrapTypeName = "cylinderObst";

static const double TwoPi = 2.0*SimTK::Pi;
static const double max_wrap_pts_circle_ang = (5.0/360.0)*TwoPi;

// The following variables could be used for speedy wrap_pts definitions (NOT CURRENTLY USED)
static const int num_circle_wrap_pts = 36;  // Number of circle points in 360 degrees
static double circle_wrap_pts_sin[num_circle_wrap_pts];
static double circle_wrap_pts_cos[num_circle_wrap_pts];
static bool circle_wrap_pts_inited = false;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
* Default constructor.
*/
WrapCylinderObst::WrapCylinderObst()
{
    initCircleWrapPts();
    constructProperties();
}

//_____________________________________________________________________________
/**
* Destructor.
*/
WrapCylinderObst::~WrapCylinderObst()
{
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/** Initialize static data variables used for speedy definition of wrap_pts (for graphics mainly) */
void WrapCylinderObst::initCircleWrapPts()
{   int i;  double q;
    for(i=0; i<num_circle_wrap_pts; i++) {
        q = TwoPi*(double)(i)/(double)(num_circle_wrap_pts);
        circle_wrap_pts_sin[i] = sin(q);
        circle_wrap_pts_cos[i] = cos(q);
    }   circle_wrap_pts_inited = true;
}

//_____________________________________________________________________________
/**
* Connect properties to local pointers.
*/
void WrapCylinderObst::constructProperties()
{
    constructProperty_radius(1.0);
    constructProperty_wrapDirection("righthand");
    m_wrapDirection = righthand;
    constructProperty_length(1.0);
}

//_____________________________________________________________________________
/*
* Perform some set up functions that happen after the
* object has been deserialized or copied.
*/
void WrapCylinderObst::extendFinalizeFromProperties()
{
    // Base class
    Super::extendFinalizeFromProperties();

    // maybe set a parent pointer, _body = aBody;
    OPENSIM_THROW_IF_FRMOBJ(
        get_radius() < 0,
        InvalidPropertyValue,
        getProperty_radius().getName(),
        "Radius cannot be less than zero");

/*
    Cylinder* cyl = new Cylinder(get_radius(), get_length());
    setGeometryQuadrants(cyl);
*/
    if (get_wrapDirection() == "righthand" || get_wrapDirection() == "right" || get_wrapDirection() == "righthanded" || get_wrapDirection() == "Righthand" || get_wrapDirection() == "Right" || get_wrapDirection() == "Righthanded")
        m_wrapDirection = righthand;
    else
    if (get_wrapDirection() == "lefthand"  || get_wrapDirection() == "left"  || get_wrapDirection() == "lefthanded"  || get_wrapDirection() == "Lefthand"  || get_wrapDirection() == "Left"  || get_wrapDirection() == "Lefthanded")
        m_wrapDirection = lefthand;
    else
    if (get_wrapDirection() == "Unassigned")  // wrapDirection was not specified in obstacle object definition; use default
        { m_wrapDirection = righthand;
        set_wrapDirection("righthand");
    }
    else {  // wrapDirection was specified incorrectly in obstacle object definition; throw an exception
        string errorMessage = "wrapDirection was specified incorrectly. "
                                "Use \"righthand\" or \"lefthand\".";
        OPENSIM_THROW_FRMOBJ(InvalidPropertyValue,
                            "wrapDirection",
                            errorMessage);
    }

    OPENSIM_THROW_IF_FRMOBJ(
        get_length() < 0,
        InvalidPropertyValue,
        getProperty_length().getName(),
        "Length cannot be less than zero");
}

//_____________________________________________________________________________
/**
 * Get the name of the type of wrap object ("cylinderObst" in this case)
 *
 * @return A string representing the type of wrap object
 */
const char* WrapCylinderObst::getWrapTypeName() const
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
string WrapCylinderObst::getDimensionsString() const
{
    stringstream dimensions;
    dimensions << "radius " << get_radius() << "\nheight " << get_length();

    return dimensions.str();
}

double WrapCylinderObst::getRadius() const {
    return get_radius();
}

void WrapCylinderObst::setRadius(double aRadius) {
    set_radius(aRadius);
}

double WrapCylinderObst::getLength() const {
    return get_length();
}

void WrapCylinderObst::setLength(double aLength) {
    set_length(aLength);
}

int WrapCylinderObst::getWrapDirection() const {
    return (int)m_wrapDirection;
}

//=============================================================================
// WRAPPING
//=============================================================================
//_____________________________________________________________________________
/**
 * Calculate the wrapping of one line segment over the cylinder oriented parallel to z-axis.
 *
 * @param aPointP One end of the line segment, already expressed in cylinder frame
 * @param aPointS The other end of the line segment, already expressed in cylinder frame
 * @param aPathWrap An object holding the parameters for this line/cylinder pairing
 * @param aWrapResult The result of the wrapping (tangent points, etc.)
 * @param aFlag A flag for indicating errors, etc.
 * @return The status, as a WrapAction enum
 */
int WrapCylinderObst::wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
                        const PathWrap& aPathWrap, WrapResult& aWrapResult, bool& aFlag) const
{
    SimTK::Vec3& aPointP = aPoint1;     double R=0.8*( m_wrapDirection==righthand ? get_radius() : -get_radius() );
    SimTK::Vec3& aPointS = aPoint2;     double Qx,Qy,Qz, Tx,Ty,Tz;

    // Initialize return values
    aFlag = false;
    aWrapResult.wrap_path_length = 0.0;
    aWrapResult.wrap_pts.setSize(0);
    
    // Compute displacements of P and S from cylinder axis
    double Px=aPointP[0], Py=aPointP[1], Pz=aPointP[2], dP=Px*Px+Py*Py, rootP=dP-R*R;
    double Sx=aPointS[0], Sy=aPointS[1], Sz=aPointS[2], dS=Sx*Sx+Sy*Sy, rootS=dS-R*R;

    // Check P and S against cylinder, and compute x and y components of wrap points Q and T
    if( rootP<0.0 || rootS<0.0 ) return insideRadius;   // One of P or S lies within the cylinder
    dP=R/dP;    rootP=sqrt(rootP);  Qx=(R*Px-rootP*Py)*dP;  Qy=(R*Py+rootP*Px)*dP;
    dS=R/dS;    rootS=sqrt(rootS);  Tx=(R*Sx+rootS*Sy)*dS;  Ty=(R*Sy-rootS*Sx)*dS;

    // Apply the 180-degree wrapping rule to see if contact is appropriate (i.e. wrap > 180 = no contact)
    if( R*(Qx*Ty-Qy*Tx) < 0.0 ) return noWrap;

    // Compute respective wrapping segment lengths
    double PQ = sqrt( (Qx-Px)*(Qx-Px) + (Qy-Py)*(Qy-Py) );
    double TS = sqrt( (Tx-Sx)*(Tx-Sx) + (Ty-Sy)*(Ty-Sy) );
    double QtoTang = acos( 1.0 - 0.5*( (Qx-Tx)*(Qx-Tx) + (Qy-Ty)*(Qy-Ty) )/(R*R) );
    double QT = R*QtoTang;
    if(QT<0.0) QT=-QT;
    
    // Assign z-axis components of wrap points Q and T
    Qz = Pz + (Sz-Pz)*(PQ) / (PQ+TS+QT);
    Tz = Sz + (Pz-Sz)*(TS) / (PQ+TS+QT);
    
    // Register results and return
    aFlag = true;
    aWrapResult.wrap_path_length = QT;  // PQ + TS + QT;
    aWrapResult.r1[0]=Qx;  aWrapResult.r1[1]=Qy;  aWrapResult.r1[2]=Qz;  
    aWrapResult.r2[0]=Tx;  aWrapResult.r2[1]=Ty;  aWrapResult.r2[2]=Tz;
    
    // Generate wrap_pts sequence of points tracing out wrapping path
    aWrapResult.wrap_pts.append(aWrapResult.r1);
    double Qang=atan2(Qy,Qx);               // Angle of point Q
    int i, n=1+ (int)((QtoTang>=0?QtoTang:-QtoTang)/max_wrap_pts_circle_ang);   // Number of angle steps from Q to T angles
    double angDelt=(QtoTang)/(double)(n);   // Delta angle for n steps from Q to T angles
    for(i=0;i<=n;i++) {
        double ang = Qang + i*angDelt;      // Angle ranging from that of Q to that of T
        SimTK::Vec3 aPointi( R*cos(ang), R*sin(ang), Qz+(Tz-Qz)*(double)(i)/(double)(n) );
        aWrapResult.wrap_pts.append(aPointi);
    }
    aWrapResult.wrap_pts.append(aWrapResult.r2);

    return wrapped;
}


