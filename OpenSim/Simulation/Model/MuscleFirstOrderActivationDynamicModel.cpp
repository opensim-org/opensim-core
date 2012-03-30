// Author: Matthew Millard
//MuscleFirstOrderActivationDynamicModel.cpp
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

//#include <SimTKcommon\Testing.h>
#include "MuscleFirstOrderActivationDynamicModel.h"

using namespace OpenSim;
using namespace SimTK;
//=============================================================================
// Code
//=============================================================================

/*
Detailed Computational Cost
        Comparison  Div.    Mult.   Add.    Assign
        3           1               1       1

*/
MuscleFirstOrderActivationDynamicModel::
        MuscleFirstOrderActivationDynamicModel(double tauActivation, 
        double tauDeactivation, double minActivation, 
        const std::string& muscleName):
        m_ta(tauActivation), m_td(tauDeactivation), 
        m_minA(minActivation)
{
    m_name = muscleName;
    m_name.append("_activation");
    double smallTol = sqrt(SimTK::Eps);
    SimTK_ERRCHK1_ALWAYS( tauActivation>smallTol && tauDeactivation>smallTol,
        "MuscleFirstOrderActivationDynamicModel::"
        "MuscleFirstOrderActivationDynamicModel",
        "%s: Activation/Deactivation time constants", m_name.c_str());

    SimTK_ERRCHK1_ALWAYS( m_minA >= 0 && m_minA < 1-smallTol,
        "MuscleFirstOrderActivationDynamicModel::"
        "MuscleFirstOrderActivationDynamicModel",
        "%s: Minimum activation must be greater than 0 and less than 1",
        m_name.c_str());

    //This simple quantity is precomputed to save a division, a 
    //subtraction and an assignment ...
    m_minAS = m_minA/(1-m_minA); 

}
        

double MuscleFirstOrderActivationDynamicModel::
    calcDerivative(double activation, double excitation) const
{
    SimTK::Array_<int> dx(1);
    dx[0] = 0;

    SimTK::Vector x(2);
    x(0) = activation;
    x(1) = excitation;

    return calcDerivative(dx,x);
}



double MuscleFirstOrderActivationDynamicModel::
    getActivationTimeConstant() const
{
    return m_ta;
}
        

double MuscleFirstOrderActivationDynamicModel::
    getDeactivationTimeConstant() const
{
    return m_td;
}
        

double MuscleFirstOrderActivationDynamicModel::getMinActivation() const
{
    return m_minA;
}


std::string MuscleFirstOrderActivationDynamicModel::getName() const
{
    return m_name;
}

double MuscleFirstOrderActivationDynamicModel::
    calcValue(const SimTK::Vector& x) const
{
    SimTK_ERRCHK1_ALWAYS(x.size() == 2,
        "MuscleFirstOrderActivationDynamicModel::calcDerivative",
        "%s: Two arguments are required: excitation and activation", 
        m_name.c_str());

    double activation = x(0);
    double excitation = x(1);

    return activation;
}

/*
Detailed Computational Cost     
        Comparison  Div.    Mult.   Add.    Assign
        9           2       2       4       5
*/
double MuscleFirstOrderActivationDynamicModel::
    calcDerivative(const SimTK::Array_<int>& derivComponents, 
                    const SimTK::Vector& x) const
{
    SimTK_ERRCHK1_ALWAYS(x.size() == 2,
        "MuscleFirstOrderActivationDynamicModel::calcDerivative",
        "%s: Two arguments are required: excitation and activation", 
        m_name.c_str());

    double da = 0;

    switch (derivComponents.size()){
        case 0:
            {
                da = calcValue(x);
            }
            break;
        case 1:
            {
                if(derivComponents[0] == 0){
                    double activation = x(0);
                    double excitation = x(1);      
                    /*
                    int epsMult = 2;
                    if((excitation >= -SimTK::Eps*epsMult) 
                                                        && (excitation < 0))
                        excitation = 0;
                    
                    if((excitation <= (1+SimTK::Eps*epsMult)) 
                                                        && (excitation > 1))
                        excitation = 1;

                    if((activation >= (m_minA-SimTK::Eps*epsMult)) 
                                                    && (activation < m_minA))
                        activation = m_minA;

                    if((activation <= (1+SimTK::Eps*epsMult)) 
                                                    && (activation > 1))
                        activation = 1;
                    */
                    /*
                    SimTK_ERRCHK1_ALWAYS(excitation >= 0 && excitation <= 1,
                    "MuscleFirstOrderActivationDynamicModel::calcDerivative",
                    "%s: Excitation must be bounded by 0 and 1",m_name.c_str());

                    SimTK_ERRCHK2_ALWAYS(activation >= m_minA && activation<=1,
                    "MuscleFirstOrderActivationDynamicModel::calcDerivative",
                    "%s: Activation must be between minA and 1 (%f and 1)",
                        m_name.c_str(), m_minA);
                    */

                    double aS = activation/(1-m_minA);
                    double tau = 0;

                    if(excitation > (aS-m_minAS)){
                        tau = m_ta*(0.5 + 1.5*(aS-m_minAS));
                    }else{
                        tau = m_td*(0.5+1.5*(aS-m_minAS));
                    }
                    da = (excitation - (aS-m_minAS))/tau;

                }else{
                   SimTK_ERRCHK1_ALWAYS(false,
                    "MuscleFirstOrderActivationDynamicModel::calcDerivative",
                    "%s: calcDerivative is only valid for the 0th partial", 
                    m_name.c_str());
                }
            }
            break;
        default:
            SimTK_ERRCHK1_ALWAYS(false,
            "MuscleFirstOrderActivationDynamicModel::calcDerivative",
            "%s: calcDerivative is only valid for the 0th and 1st derivative", 
            m_name.c_str());
            
    }

    return da;
}

int MuscleFirstOrderActivationDynamicModel::getArgumentSize() const
{
    return 2; // excitation and activation
}

int MuscleFirstOrderActivationDynamicModel::
    getMaxDerivativeOrder() const
{
    return 1;
}