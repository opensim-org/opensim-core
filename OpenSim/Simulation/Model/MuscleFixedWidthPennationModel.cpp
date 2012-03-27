//MuscleFixedWidthPennationModel.cpp
// Author: Matthew Millard
/*
 * Permission is hereby granted, free of charge, to any person obtaining a    *
 * copy of this software and associated documentation files (the "Software"), *
 * to deal in the Software without restriction, including without limitation  *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,   *
 * and/or sell copies of the Software, and to permit persons to whom the      *
 * Software is furnished to do so, subject to the following conditions:       *
 *                                                                            *
 * The above copyright notice and this permission notice shall be included in *
 * all copies or substantial portions of the Software.                        *
 *                                                                            *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS, CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,    *
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR      *
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE  *
 * USE OR OTHER DEALINGS IN THE SOFTWARE.                                     *
 * -------------------------------------------------------------------------- */
//=============================================================================
// INCLUDES
//=============================================================================
#include "MuscleFixedWidthPennationModel.h"
using namespace OpenSim;
using namespace SimTK;
using namespace std;

MuscleFixedWidthPennationModel::
    MuscleFixedWidthPennationModel(double optimalFiberLength,
                                          double optimalPennationAngle,
                                          std::string& caller)
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

    m_optimalFiberLength    = optimalFiberLength;
    m_optimalPennationAngle = optimalPennationAngle;

    m_parallelogramHeight = optimalFiberLength * sin(optimalPennationAngle);
}

double MuscleFixedWidthPennationModel::getParallelogramHeight()
{
    return m_parallelogramHeight;
}

double MuscleFixedWidthPennationModel::getOptimalFiberLength()
{
    return m_optimalFiberLength;
}

double MuscleFixedWidthPennationModel::getOptimalPennationAngle()
{
    return m_optimalPennationAngle;
}
/*==============================================================================
Position level kinematics
==============================================================================*/
double MuscleFixedWidthPennationModel::
                                    calcPennationAngle(double fiberLength,
                                                        std::string& caller)
{
    //lce_opt * sin(phi_opt) = m_parallelogramHeight
    //sin(phi_opt) = m_parallelogramHeight/lce
        SimTK_ERRCHK1_ALWAYS( fiberLength > 0,
            "MuscleFixedWidthPennationModel::calcPennationAngle",
            "%s: Fiber length cannot be zero.",caller.c_str());

        double sin_phi = m_parallelogramHeight/fiberLength;

        SimTK_ERRCHK1_ALWAYS( sin_phi <= 1,
            "MuscleFixedWidthPennationModel::calcPennationAngle",
            "%s: Fiber length is too short.",
            caller.c_str());

        double phi = asin(sin_phi);
    
    return phi;

}

double MuscleFixedWidthPennationModel::
    calcTendonLength(double cosPennationAngle,
                        double fiberLength, 
                        double muscleLength)
{
    double tl = muscleLength - fiberLength*cosPennationAngle;
    return tl;
}

/*==============================================================================
Velocity level kinematics
==============================================================================*/
   
double MuscleFixedWidthPennationModel::
    calcPennationAngularVelocity(double tanPennationAngle,
                                    double fiberLength,  
                                    double fiberVelocity,
                                    std::string& caller)
{
            
    SimTK_ERRCHK1_ALWAYS( fiberLength > 0,
        "MuscleFixedWidthPennationModel::calcPennationAngularVelocity",
        "%s: Fiber length cannot be zero.",caller.c_str());

    double dphi = -(fiberVelocity/fiberLength)*tanPennationAngle;
    return dphi;

}

double MuscleFixedWidthPennationModel::
    calcTendonVelocity(double cosPennationAngle,
                            double sinPennationAngle,
                            double pennationAngularVelocity,
                            double fiberLength,    
                            double fiberVelocity,
                            double muscleVelocity)
{
    double tendonVelocity = muscleVelocity - fiberVelocity * cosPennationAngle
            + fiberLength * sinPennationAngle * pennationAngularVelocity;
    return tendonVelocity;
}


/*==============================================================================
Kinematic Position Level
Partial Derivatives w.r.t. fiber length 
==============================================================================*/

double MuscleFixedWidthPennationModel::
    calc_DpennationAngle_DfiberLength(double fiberLength, std::string& caller)
{
    //SimTK_ERRCHK1_ALWAYS( fiberLength > 0,
    //"MuscleFixedWidthPennationModel::calc_DpennationAngle_DfiberLength",
    // "%s: Fiber length cannot be zero.",caller.c_str());

    SimTK_ERRCHK1_ALWAYS( fiberLength > m_parallelogramHeight ,
     "MuscleFixedWidthPennationModel::calc_DpennationAngle_DfiberLength",
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
    calc_DtendonLength_DfiberLength(double fiberLength, 
                                    double sinPennationAngle,
                                    double cosPennationAngle,
                                    double DpennationAngle_DfiberLength,                                    
                                    std::string& caller)
{
    //SimTK_ERRCHK1_ALWAYS( fiberLength > 0,
    //"MuscleFixedWidthPennationModel::calc_DpennationAngle_DfiberLength",
    //"%s: Fiber length cannot be zero.",caller.c_str());

    SimTK_ERRCHK1_ALWAYS( fiberLength > m_parallelogramHeight ,
     "MuscleFixedWidthPennationModel::calc_DpennationAngle_DfiberLength",
     "%s: Fiber length is below the minimum bound for this muscle.",
     caller.c_str() );


    //double dtl_dlce = DmuscleLength_DfiberLength-cosPennationAngle 
    //    + fiberLength * sinPennationAngle * DpennationAngle_DfiberLength;

    double dtl_dlce = -cosPennationAngle 
        + fiberLength * sinPennationAngle * DpennationAngle_DfiberLength;
    return dtl_dlce;
}

