/* -------------------------------------------------------------------------- *
 *                OpenSim:  MuscleFixedWidthPennationModel.cpp                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
//=============================================================================
// INCLUDES
//=============================================================================
#include "MuscleFixedWidthPennationModel.h"
using namespace OpenSim;
using namespace SimTK;
using namespace std;

void MuscleFixedWidthPennationModel::setNull()
{
    m_parallelogramHeight   = SimTK::NaN;
}

void MuscleFixedWidthPennationModel::constructProperties()
{
    constructProperty_optimal_fiber_length(SimTK::NaN);
    constructProperty_optimal_pennation_angle(SimTK::NaN);
}


MuscleFixedWidthPennationModel::MuscleFixedWidthPennationModel()
{
    setNull();
    constructProperties();
}

MuscleFixedWidthPennationModel::
    MuscleFixedWidthPennationModel(double optimalFiberLength,
            double optimalPennationAngle,std::string& caller)
{
    SimTK_ERRCHK1_ALWAYS( optimalFiberLength > 0,
        "MuscleFixedWidthPennationModel::"
        "MuscleFixedWidthPennationModel",
        "%s: optimal fiber length must be greater than zero.",caller.c_str());
    
    SimTK_ERRCHK1_ALWAYS( optimalPennationAngle < SimTK::Pi/2 
                       && optimalPennationAngle >= 0,
        "MuscleFixedWidthPennationModel::"
        "MuscleFixedWidthPennationModel",
        "%s: optimal pennation angle be between 0 and Pi/2 radians.",
        caller.c_str());

    setNull();
    constructProperties();

    set_optimal_fiber_length(optimalFiberLength);
    set_optimal_pennation_angle(optimalPennationAngle);

    m_parallelogramHeight    = optimalFiberLength * sin(optimalPennationAngle);

    //All of the below is to establish a lower bound on fiber length that is
    //close to the minimum value. The absolute minimum fiber length cannot be
    //used in many of the calculations in this model as it causes numerical
    //singularities, or near-singularities
    m_minimumFiberLength = m_parallelogramHeight + 0.001*optimalFiberLength;
    m_maximumSinPennation    = m_parallelogramHeight
                            /(m_minimumFiberLength);
    m_maximumPennationAngle = asin(m_maximumSinPennation);

    m_minimumFiberLengthAlongTendon 
        =  m_minimumFiberLength*cos(m_maximumPennationAngle);
}

void MuscleFixedWidthPennationModel::ensurePropertiesSet() const
{
    double optFibLen = get_optimal_fiber_length();
    double optPenAng = get_optimal_pennation_angle();
    
    SimTK_ERRCHK1_ALWAYS((isnan(optFibLen) != true && isnan(optPenAng) != true),
    "MuscleFixedWidthPennationModel",
    "%s: Properties have not yet been set!.",getName().c_str());

}


double MuscleFixedWidthPennationModel::getParallelogramHeight() const
{
    ensurePropertiesSet();
    return m_parallelogramHeight;
}

double MuscleFixedWidthPennationModel::getOptimalFiberLength() const
{
    ensurePropertiesSet();
    return get_optimal_fiber_length();
}

double MuscleFixedWidthPennationModel::getMinimumFiberLength() const
{
    ensurePropertiesSet();
    return m_minimumFiberLength;
}


double MuscleFixedWidthPennationModel::
    getMinimumFiberLengthAlongTendon() const
{
    ensurePropertiesSet();
    return m_minimumFiberLengthAlongTendon;
}

double MuscleFixedWidthPennationModel::
    clampFiberLength(double fiberLength) const
{
    ensurePropertiesSet();
    double clampedFiberLength = 
        max(m_minimumFiberLength, fiberLength);


    return clampedFiberLength;
}

double MuscleFixedWidthPennationModel::getOptimalPennationAngle() const
{
    ensurePropertiesSet();
    return get_optimal_pennation_angle();
}
/*==============================================================================
Position level kinematics
==============================================================================*/
double MuscleFixedWidthPennationModel::calcPennationAngle(double fiberLength,
                                                      std::string& caller) const
{
    double phi = 0;
    

    //This computation is only worth performing if the fiber has a pennation
    //angle
    ensurePropertiesSet();

    double optimalPennationAngle = get_optimal_pennation_angle();

    if(optimalPennationAngle > SimTK::Eps){
        //lce_opt * sin(phi_opt) = m_parallelogramHeight
        //sin(phi_opt) = m_parallelogramHeight/lce

        //These exceptions were causing real headaches, so we're saturating 
        //the output when illeagle fiber lengths are passed in rather than
        //throwing an exception.

            //SimTK_ERRCHK1_ALWAYS( fiberLength > 0,
            //    "MuscleFixedWidthPennationModel::calcPennationAngle",
            //    "%s: Fiber length cannot be zero.",caller.c_str());
       
            //SimTK_ERRCHK1_ALWAYS( sin_phi <= 1,
            //    "MuscleFixedWidthPennationModel::calcPennationAngle",
            //    "%s: Fiber length is too short, and is creating an "
            //    " unphysical pennation angle.",
            //    caller.c_str());

        if(fiberLength > m_minimumFiberLength){
            double sin_phi = m_parallelogramHeight/fiberLength;
            
            if(sin_phi < m_maximumSinPennation){
                phi = asin(sin_phi);
            }else{
                phi = m_maximumPennationAngle;

                //Log a warning: commented out because these warnings
                //utterly swamp the command window
                /*cerr << endl;
                cerr << "Warning: MuscleFixedWidthPennationModel::"
                        "calcPennationAngle"
                     << endl;

                cerr <<"    :"<<caller.c_str() << endl;
                cerr <<"    :"<<"Pennation angle is undefined as sin(phi) > 1."
                     << endl;
                cerr <<" Clamping to pennation angle to"
                        " SimTK::Pi/2 and continuing" 
                     << endl;
                cerr << endl;
                */

            }            
        }else{
            phi = m_maximumPennationAngle;

            //Log a warning: commented out because these warnings
            //utterly swamp the command window
            /*cerr << endl;
            cerr << "Warning: MuscleFixedWidthPennationModel::"
                    "calcPennationAngle"
                 << endl;

            cerr <<"    :" << caller.c_str() << endl;
            cerr <<"    :Pennation angle is undefined as"
                        " fiber length is negative." 
                 << endl;
            cerr <<"    :Clamping pennation angle to SimTK::Pi/2 and continuing" 
                 << endl;
            cerr << endl;
            */
        }
    }
    return phi;

}

double MuscleFixedWidthPennationModel::
    calcTendonLength(double cosPennationAngle, double fiberLength, 
                                               double muscleLength) const
{
    double tl = muscleLength - fiberLength*cosPennationAngle;
    return tl;
}

double MuscleFixedWidthPennationModel::
  calcFiberLengthAlongTendon(double fiberLength, double cosPennationAngle) const
{
    return fiberLength*cosPennationAngle;
}

/*==============================================================================
Velocity level kinematics
==============================================================================*/


double MuscleFixedWidthPennationModel::
    calcPennationAngularVelocity(double tanPennationAngle, double fiberLength,  
                                double fiberVelocity, std::string& caller) const
{
    double dphi=0;
    ensurePropertiesSet();
    double optimalPennationAngle = get_optimal_pennation_angle();

    if(optimalPennationAngle > SimTK::Eps){
        SimTK_ERRCHK1_ALWAYS( fiberLength > 0,
            "MuscleFixedWidthPennationModel::calcPennationAngularVelocity",
            "%s: Fiber length cannot be zero.",caller.c_str());

        dphi = -(fiberVelocity/fiberLength)*tanPennationAngle;
    }
    return dphi;

}

double MuscleFixedWidthPennationModel::
    calcTendonVelocity(double cosPennationAngle,
                            double sinPennationAngle,
                            double pennationAngularVelocity,
                            double fiberLength,    
                            double fiberVelocity,
                            double muscleVelocity) const
{
    double tendonVelocity = muscleVelocity - fiberVelocity * cosPennationAngle
            + fiberLength * sinPennationAngle * pennationAngularVelocity;
    return tendonVelocity;
}

double MuscleFixedWidthPennationModel::
  calcFiberVelocityAlongTendon(double fiberLength, double fiberVelocity,
                     double sinPennationAngle, double cosPennationAngle, 
                                        double pennationAngularVelocity) const
{
    //double dlceAT = dlce*cos(phi) - lce*sin(phi)*dphidt;
    double dlceAT =     fiberVelocity*cosPennationAngle 
                       -fiberLength*sinPennationAngle*pennationAngularVelocity;
    return dlceAT;
}
/*==============================================================================
Acceleration Level
==============================================================================*/

double MuscleFixedWidthPennationModel::
    calcPennationAngularAcceleration(double fiberLength, 
                                        double fiberVelocity,
                                        double fiberAcceleration,
                                        double sinPennationAngle,
                                        double cosPennationAngle,
                                    double pennationAngularVelocity,
                                    std::string& caller) const
{

    SimTK_ERRCHK1_ALWAYS( fiberLength > 0,
        "MuscleFixedWidthPennationModel::calcPennationAngularAcceleration",
        "%s: Fiber length cannot be zero.",caller.c_str());
    SimTK_ERRCHK1_ALWAYS( cosPennationAngle > 0,
        "MuscleFixedWidthPennationModel::calcPennationAngularAcceleration",
        "%s: cosPennationAngle cannot be zero.",caller.c_str());

    double lce      = fiberLength;
    double dlce_dt  = fiberVelocity;
    double ddlce_dtt= fiberAcceleration;

    double sinphi = sinPennationAngle;
    double cosphi = cosPennationAngle;
    double dphi_dt= pennationAngularVelocity;
    
    //ddphi_dtt
    double t1 = sinphi;
    double t3 = cosphi;
    double t8 = dphi_dt * dphi_dt;
    double ddphi_dtt = -(ddlce_dtt * t1 + 0.2e1 * dlce_dt * t3 * dphi_dt 
                        - lce * t1 * t8) / lce / t3;

    return ddphi_dtt;
}

double MuscleFixedWidthPennationModel::
    calcFiberAccelerationAlongTendon(double fiberLength, 
                                        double fiberVelocity,
                                        double fiberAcceleration,
                                        double sinPennationAngle,
                                        double cosPennationAngle,
                                    double pennationAngularVelocity,
                                    double pennationAngularAcceleration) const
{

    double lce      = fiberLength;
    double dlce_dt  = fiberVelocity;
    double ddlce_dtt= fiberAcceleration;

    double sinphi   = sinPennationAngle;
    double cosphi   = cosPennationAngle;
    double dphi_dt  = pennationAngularVelocity;
    double ddphi_dtt= pennationAngularAcceleration;
    
    //ddlceAT_dtt
    double t1 = cosphi;
    double t3 = sinphi;
    double t8 = dphi_dt * dphi_dt;
    double ddlceAT_dtt = ddlce_dtt * t1 - 0.2e1 * dlce_dt * t3 * dphi_dt 
                        - lce * t1 * t8 - lce * t3 * ddphi_dtt;

    return ddlceAT_dtt;
}


/*==============================================================================
Kinematic Position Level
Partial Derivatives w.r.t. fiber length 
==============================================================================*/
double MuscleFixedWidthPennationModel::
    calc_DFiberLengthAlongTendon_DfiberLength(double fiberLength,                                     
                                         double sinPennationAngle, 
                                         double cosPennationAngle,                                    
                                         double DpennationAngle_DfiberLength
                                         ) const
{
    //Renaming to smaller names, else the equations become unreadable
    double lce      = fiberLength;
    double sinPhi   = sinPennationAngle;
    double cosPhi   = cosPennationAngle;
    double Dphi_Dlce=DpennationAngle_DfiberLength;

    //d/dlce (lce*cos(phi)) = cos(phi) - lce*sin(phi)*dphi_dlce 
    double DlceAT_Dlce = cosPhi - lce*sinPhi*Dphi_Dlce;
    return DlceAT_Dlce;
}


double MuscleFixedWidthPennationModel::
  calc_DPennationAngularVelocity_DfiberLength(double fiberLength, 
                                              double fiberVelocity,
                                              double sinPennationAngle, 
                                              double cosPennationAngle, 
                                          double pennationAngularVelocity,
                                          double DpennationAngle_DfiberLength,
                                          std::string& caller
                                          ) const
{
    //Variable name switches to make the math more easily read
    double lce = fiberLength;
    double dlce = fiberVelocity;
    double sinPhi = sinPennationAngle;
    double cosPhi = cosPennationAngle;
    double dphidt = pennationAngularVelocity;
    double Dphi_Dlce = DpennationAngle_DfiberLength;

    SimTK_ERRCHK1_ALWAYS( cosPhi > SimTK::Eps,
    "MuscleFixedWidthPennationModel::calc_DFiberVelocityAlongTendon_DfiberLength",
    "%s: Pennation angle is 90 degrees.",caller.c_str());

    double tanPhi = sinPhi/cosPhi;    
    double DtanPhi_Dlce = (1+tanPhi*tanPhi)*Dphi_Dlce;

    //dphidt = -(fiberVelocity/fiberLength)*tanPennationAngle;
    //dphidt = -(dlce/lce)*tanPhi;
    
    SimTK_ERRCHK1_ALWAYS( lce > SimTK::Eps,
    "MuscleFixedWidthPennationModel::calc_DFiberVelocityAlongTendon_DfiberLength",
    "%s: Fiber length is close to 0.",caller.c_str());
    double Ddphidt_Dlce= (dlce/(lce*lce))*tanPhi
                        -(dlce/lce)*DtanPhi_Dlce;
    return Ddphidt_Dlce;
}

double MuscleFixedWidthPennationModel::
  calc_DFiberVelocityAlongTendon_DfiberLength(
                                double fiberLength, 
                                double fiberVelocity,
                                double sinPennationAngle, 
                                double cosPennationAngle,
                                double pennationAngularVelocity,
                                double DpennationAngle_DfiberLength,
                                double DpennationAngularVelocity_DfiberLength
                                ) const
{
    //Variable name switches to make the math more easily read
    double lce = fiberLength;
    double dlce = fiberVelocity;
    double sinPhi = sinPennationAngle;
    double cosPhi = cosPennationAngle;
    double dphidt = pennationAngularVelocity;
    double Dphi_Dlce = DpennationAngle_DfiberLength;
    double Ddphidt_Dlce= DpennationAngularVelocity_DfiberLength;

    //double dlceAT = dlce*cos(phi) - lce*sin(phi)*dphidt;
    //Now taking the partial derivative of dlceAT w.r.t lce
    double DdlceAT_Dlce = dlce*(-sinPhi*Dphi_Dlce)
                          -(1)*sinPhi*dphidt
                          -lce*(cosPhi*Dphi_Dlce)*dphidt
                          -lce*sinPhi*(Ddphidt_Dlce);    
    return DdlceAT_Dlce;
}

double MuscleFixedWidthPennationModel::
    calc_DPennationAngle_DfiberLength(double fiberLength, 
                                      std::string& caller) const
{
    ensurePropertiesSet();

    SimTK_ERRCHK1_ALWAYS( fiberLength > m_parallelogramHeight ,
     "MuscleFixedWidthPennationModel::calc_DPennationAngle_DfiberLength",
     "%s: Fiber length is below the minimum bound for this muscle.",
     caller.c_str() );

    // phi = asin( parallelogramHeight/lce)
    //d_phi/d_lce = d/dlce (asin(parallelogramHeight/lce))

    double t1 = fiberLength*fiberLength;
    double t2 = 1 / t1;
    double t4 = (m_parallelogramHeight*m_parallelogramHeight);
    double t7 = sqrt((1 - t4 * t2));
    double dphi_dlce = -m_parallelogramHeight * t2 / t7;

    return dphi_dlce;
}
                

double MuscleFixedWidthPennationModel::
    calc_DTendonLength_DfiberLength(double fiberLength, 
                                    double sinPennationAngle,
                                    double cosPennationAngle,
                                    double DpennationAngle_DfiberLength,                                    
                                    std::string& caller)  const
{
    ensurePropertiesSet();

    SimTK_ERRCHK1_ALWAYS( fiberLength > m_parallelogramHeight ,
     "MuscleFixedWidthPennationModel::calc_DTendonLength_DfiberLength",
     "%s: Fiber length is below the minimum bound for this muscle.",
     caller.c_str() );


    //double dtl_dlce = DmuscleLength_DfiberLength-cosPennationAngle 
    //    + fiberLength * sinPennationAngle * DpennationAngle_DfiberLength;

    double dtl_dlce = -cosPennationAngle 
        + fiberLength * sinPennationAngle * DpennationAngle_DfiberLength;
    return dtl_dlce;
}

/*==============================================================================
Kinematic Fiber Pose Equations
Useful during initialization
==============================================================================*/

double MuscleFixedWidthPennationModel::
    calcFiberVelocity(  double fiberLength,
                        double sinPennation,
                        double cosPennation,                        
                        double muscleLength,
                        double tendonLength,
                        double muscleVelocity, 
                        double tendonVelocity,                                     
                        std::string& caller) const
{
    double t2 = cosPennation * cosPennation;
    double t6 = sinPennation * sinPennation;
    double denominator = (t2*cosPennation*fiberLength + 
                          t6*muscleLength - t6*tendonLength);

    double fiberVelocity = SimTK::NaN;

    //SimTK_ERRCHK1_ALWAYS( denominator >= SimTK::Eps ,
    // "MuscleFixedWidthPennationModel::calcFiberVelocity",
    // "%s: Equation is singular: check pennation angle",
    // caller.c_str() );

    if(abs(denominator) > SimTK::Eps){
        fiberVelocity = (muscleVelocity - tendonVelocity)*t2*fiberLength 
                            / denominator;
    }
    return fiberVelocity;
}

double MuscleFixedWidthPennationModel::
    calcFiberLength(  double muscleLength, 
                    double tendonLength, 
                    std::string& caller) const
{
    double fiberLengthAT = muscleLength-tendonLength;
    //SimTK_ERRCHK1_ALWAYS( fiberLengthAT >= SimTK::Eps ,
    // "MuscleFixedWidthPennationModel::calcFiberLength",
    // "%s: Equation is singular: pennation angle of 90 predicted",
    // caller.c_str() );
    //SimTK::Vec2 pose;

    double fiberLength = 0;

    if(fiberLengthAT >= getMinimumFiberLengthAlongTendon()){
        //double tanPhi = m_parallelogramHeight/fiberLengthAT;    
        //double phi = atan(tanPhi);
        fiberLength = sqrt(m_parallelogramHeight*m_parallelogramHeight
                                  +fiberLengthAT*fiberLengthAT);
    }else{
        fiberLength = getMinimumFiberLength();
    }

    return fiberLength;
}
