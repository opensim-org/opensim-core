/* -------------------------------------------------------------------------- *
 *            OpenSim:  MuscleSecondOrderActivationDynamicModel.cpp            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
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
//=============================================================================
// INCLUDES
//=============================================================================
#include "MuscleSecondOrderActivationDynamicModel.h"
using namespace std;
using namespace OpenSim;
using namespace SimTK;
//=============================================================================
// Code
//=============================================================================
MuscleSecondOrderActivationDynamicModel::
    MuscleSecondOrderActivationDynamicModel()
{
    setName("default_MuscleSecondOrderActivationDynamicModel");
    setNull();
    constructProperties();
    buildModel();
}

void MuscleSecondOrderActivationDynamicModel::ensureModelUpToDate()
{
    if(isObjectUpToDateWithProperties()==false){
        buildModel();
    }

}

void MuscleSecondOrderActivationDynamicModel::buildModel()
{         
    setObjectIsUpToDateWithProperties();
}

/*
Detailed Computational Cost
        Comparison  Div.    Mult.   Add.    Assign
        3           1               1       1

*/
MuscleSecondOrderActivationDynamicModel::
        MuscleSecondOrderActivationDynamicModel(double twitchTimeConstant, 
                                                double minActivation, 
                                                const std::string& muscleName)
{
    setNull();
    constructProperties();

    std::string name = muscleName;
    name.append("_activation");
    setName(name);

   
    SimTK_ERRCHK1_ALWAYS( twitchTimeConstant>SimTK::SignificantReal ,
        "MuscleSecondOrderActivationDynamicModel::"
        "MuscleSecondOrderActivationDynamicModel",
        "%s: twitch time constants must be > 0", name.c_str());

    SimTK_ERRCHK1_ALWAYS( minActivation >= 0 
                            && minActivation < 1-SimTK::SignificantReal,
        "MuscleSecondOrderActivationDynamicModel::"
        "MuscleSecondOrderActivationDynamicModel",
        "%s: Minimum activation must be greater than 0 and less than 1",
        name.c_str());

    set_twitch_time_constant(twitchTimeConstant);
    set_minimum_activation(minActivation);
    buildModel();    
}
        
void MuscleSecondOrderActivationDynamicModel::setNull()
{
    setAuthors("Matthew Millard");    
}

void MuscleSecondOrderActivationDynamicModel::constructProperties()
{
    constructProperty_twitch_time_constant(0.050);
    constructProperty_minimum_activation(0.01);    
}

double MuscleSecondOrderActivationDynamicModel::
    calcDerivative( double dactivation_dt,
                    double activation,  
                    double excitation) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "MuscleSecondOrderActivationDynamicModel: object is not"
        " to date with properties");

    SimTK::Array_<int> dx(2);
    dx[0] = 0;
    dx[1] = 0;

    SimTK::Vector x(3);
    x(0) = dactivation_dt;
    x(1) = activation;
    x(2) = excitation;

    return calcDerivative(dx,x);
}



double MuscleSecondOrderActivationDynamicModel::
    getTwitchTimeConstant() const
{
    return get_twitch_time_constant();
}
        


double MuscleSecondOrderActivationDynamicModel::getMinimumActivation() const
{
    return get_minimum_activation();
}

double MuscleSecondOrderActivationDynamicModel::getMaximumActivation() const
{
    return 1.0;
}

double MuscleSecondOrderActivationDynamicModel::
    clampActivation(double activation) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "MuscleSecondOrderActivationDynamicModel: object is not"
        " to date with properties");

    //Clamp the lower bound
    double clampedActivation = max(get_minimum_activation(), activation);
    //Clamp the upper bound
    clampedActivation = min(1.0, clampedActivation);
    return clampedActivation;
}

bool MuscleSecondOrderActivationDynamicModel::
    setTwitchTimeConstant(double activationTimeConstant) 
{

    if(activationTimeConstant > SimTK::SignificantReal){
        set_twitch_time_constant(activationTimeConstant);
        buildModel();
        return true;
    }else{
        return false;
    }
}
        
bool MuscleSecondOrderActivationDynamicModel::
    setMinimumActivation(double minimumActivation)
{

    if(minimumActivation >= 0.0){
        set_minimum_activation(minimumActivation);
        buildModel();
        return true;
    }else{
        return false;
    }
}

double MuscleSecondOrderActivationDynamicModel::
    calcValue(const SimTK::Vector& x) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "MuscleSecondOrderActivationDynamicModel: object is not"
        " to date with properties");

    SimTK_ERRCHK1_ALWAYS(x.size() == 3,
        "MuscleSecondOrderActivationDynamicModel::calcDerivative",
        "%s: Two arguments are required: excitation and activation", 
        getName().c_str());

    // double dactivation_dt   = x(0);
    double activation       = x(1);
    // double excitation       = x(2);

    return activation;
}

/*
Detailed Computational Cost     
        Comparison  Div.    Mult.   Add.    Assign
        9           2       2       4       5
*/
double MuscleSecondOrderActivationDynamicModel::
    calcDerivative(const SimTK::Array_<int>& derivComponents, 
                    const SimTK::Vector& x) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "MuscleSecondOrderActivationDynamicModel: object is not"
        " to date with properties");

    SimTK_ERRCHK1_ALWAYS(x.size() == 3,
        "MuscleSecondOrderActivationDynamicModel::calcDerivative",
        "%s: Three arguments are required: "
        "dactivation_dt, activation, excitation", 
        getName().c_str());

    double nDa = 0; //Nth derivative of a

    switch (derivComponents.size()){
        case 0:
            {
                nDa = x(1); //return activation
            }
            break;
        case 1: 
            {
                nDa = x(0); //return dactivation_dt
            }
            break;
        case 2:
            {
                if(derivComponents[0] == 0 && derivComponents[1] == 0){
                    double da_dt  = x(0);
                    double a      = x(1);
                    double u      = x(2);      

                    double cu = max(0.0, u);
                    cu        = min(1.0, cu);

                    double minAct   = get_minimum_activation();                    
                    double omega    = 1/get_twitch_time_constant();

                    if(da_dt < 0){
                        omega = omega*2.0;
                    }

                    double omega2   = omega*omega;
                    double ca       = clampActivation(a);

                    double aS       = ca/(1-minAct);
                    double minAS    = minAct/(1-minAct);
                    
                    double zeta = 1.0; //Critically damped system

                    nDa = cu*omega2 
                        - (2*zeta*omega*da_dt 
                            + (aS-minAS)*omega2);                                       

                }else{
                   SimTK_ERRCHK1_ALWAYS(false,
                    "MuscleSecondOrderActivationDynamicModel::calcDerivative",
                    "%s: calcDerivative is only valid for the 0th partial", 
                    getName().c_str());
                }
            }
            break;
        default:
            SimTK_ERRCHK1_ALWAYS(false,
            "MuscleSecondOrderActivationDynamicModel::calcDerivative",
            "%s: calcDerivative is only valid for the 0th, 1st, and "
            "2nd derivative", 
            getName().c_str());
            
    }

    return nDa;
}

int MuscleSecondOrderActivationDynamicModel::getArgumentSize() const
{
    return 3; // dactivation_dt, activation, excitation
}

int MuscleSecondOrderActivationDynamicModel::
    getMaxDerivativeOrder() const
{
    return 2;
}

