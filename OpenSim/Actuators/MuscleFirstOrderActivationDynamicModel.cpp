/* -------------------------------------------------------------------------- *
 *            OpenSim:  MuscleFirstOrderActivationDynamicModel.cpp            *
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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
//=============================================================================
// INCLUDES
//=============================================================================

//#include <SimTKcommon\Testing.h>
#include "MuscleFirstOrderActivationDynamicModel.h"
using namespace std;
using namespace OpenSim;
using namespace SimTK;
//=============================================================================
// Code
//=============================================================================
MuscleFirstOrderActivationDynamicModel::
    MuscleFirstOrderActivationDynamicModel():m_minAS(SimTK::NaN)
{
    setName("default_MuscleFirstOrderActivationDynamicModel");
    setNull();
    constructProperties();
}

void MuscleFirstOrderActivationDynamicModel::buildModel()
{
    //This simple quantity is precomputed to save a division, a 
    //subtraction and an assignment ...
    double minActivation = get_minimum_activation();
    m_minAS = minActivation/(1-minActivation); 
}

/*
Detailed Computational Cost
        Comparison  Div.    Mult.   Add.    Assign
        3           1               1       1

*/
MuscleFirstOrderActivationDynamicModel::
        MuscleFirstOrderActivationDynamicModel(double tauActivation, 
        double tauDeactivation, double minActivation, 
        const std::string& muscleName)
{
    setNull();
    constructProperties();

    std::string name = muscleName;
    name.append("_activation");
    setName(name);

   
    SimTK_ERRCHK1_ALWAYS( tauActivation>SimTK::SignificantReal 
                            && tauDeactivation>SimTK::SignificantReal,
        "MuscleFirstOrderActivationDynamicModel::"
        "MuscleFirstOrderActivationDynamicModel",
        "%s: Activation/Deactivation time constants", name.c_str());

    SimTK_ERRCHK1_ALWAYS( minActivation >= 0 
                            && minActivation < 1-SimTK::SignificantReal,
        "MuscleFirstOrderActivationDynamicModel::"
        "MuscleFirstOrderActivationDynamicModel",
        "%s: Minimum activation must be greater than 0 and less than 1",
        name.c_str());

    set_activation_time_constant(tauActivation);
    set_deactivation_time_constant(tauDeactivation);
    set_minimum_activation(minActivation);
    buildModel();
    

}
        
void MuscleFirstOrderActivationDynamicModel::setNull()
{
	setAuthors("Matthew Millard");
    m_minAS = SimTK::NaN;
}

void MuscleFirstOrderActivationDynamicModel::constructProperties()
{
    constructProperty_activation_time_constant(0.010);
    constructProperty_deactivation_time_constant(0.040);
    //constructProperty_minimum_activation(0.01);
    //m_minAS = get_minimum_activation()/(1-get_minimum_activation());
    constructProperty_minimum_activation(0.01);
    buildModel();
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
    return get_activation_time_constant();
}
        

double MuscleFirstOrderActivationDynamicModel::
    getDeactivationTimeConstant() const
{
    return get_deactivation_time_constant();
}
        

double MuscleFirstOrderActivationDynamicModel::getMinimumActivation() const
{
    return get_minimum_activation();
}

double MuscleFirstOrderActivationDynamicModel::getMaximumActivation() const
{
    return 1.0;
}

double MuscleFirstOrderActivationDynamicModel::
    clampActivation(double activation) const
{
    //Clamp the lower bound
    double clampedActivation = max(get_minimum_activation(), activation);
    //Clamp the upper bound
    clampedActivation = min(1.0, clampedActivation);
    return clampedActivation;
}

bool MuscleFirstOrderActivationDynamicModel::
    setActivationTimeConstant(double activationTimeConstant) 
{

    if(activationTimeConstant > SimTK::SignificantReal){
        set_activation_time_constant(activationTimeConstant);
        buildModel();
        return true;
    }else{
        return false;
    }
}
        

bool MuscleFirstOrderActivationDynamicModel::
    setDeactivationTimeConstant(double deactivationTimeConstant) 
{

    if(deactivationTimeConstant > SimTK::SignificantReal){
        set_deactivation_time_constant(deactivationTimeConstant);
        buildModel();
        return true;
    }else{
        return false;
    }
}
        

bool MuscleFirstOrderActivationDynamicModel::
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

double MuscleFirstOrderActivationDynamicModel::
    calcValue(const SimTK::Vector& x) const
{
    SimTK_ERRCHK1_ALWAYS(x.size() == 2,
        "MuscleFirstOrderActivationDynamicModel::calcDerivative",
        "%s: Two arguments are required: excitation and activation", 
        getName().c_str());

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
        getName().c_str());

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
                   
                    //This exception was causing problems, so we're clamping
                    //excitation
                    //SimTK_ERRCHK1_ALWAYS(excitation >= 0 && excitation <= 1,
                    //"MuscleFirstOrderActivationDynamicModel::calcDerivative",
                    //"%s: Excitation must be bounded by 0 and 1",getName().c_str());

                    double clampedExcitation = max(0.0, excitation);
                    clampedExcitation        = min(1.0, clampedExcitation);

                    double minAct = get_minimum_activation();

                    //SimTK_ERRCHK2_ALWAYS(activation >= minAct && activation<=1,
                    //"MuscleFirstOrderActivationDynamicModel::calcDerivative",
                    //"%s: Activation must be between minA and 1 (%f and 1)",
                    //    getName().c_str(), minAct);
                    double clampedActivation = clampActivation(activation);


                    double aS = clampedActivation/(1-minAct);
                    double tau = 0;

                    double ta = get_activation_time_constant();
                    double td = get_deactivation_time_constant();

                    if(clampedExcitation > (aS-m_minAS)){
                        tau = ta*(0.5 + 1.5*(aS-m_minAS));
                    }else{
                        tau = td*(0.5+1.5*(aS-m_minAS));
                    }
                    da = (clampedExcitation - (aS-m_minAS))/tau;

                    //Log warnings : commented out because these warnings
                    //utterly swamp the command window.
                    /*
                    if(clampedExcitation != excitation){
                        cerr << endl;
                        cerr << "MuscleFirstOrderActivationDynamicModel::"
                                "calcDerivative"
                             << endl;

                        cerr <<"    :" << getName().c_str() << endl;
                        cerr <<"    :Excitation was outside of 0-1. " << endl;
                        cerr <<"    :Clamping excitation to 0-1 and continuing" 
                             << endl;
                        cerr << endl;
                    }

                    if(activation != clampedActivation){
                        cerr << endl;
                        cerr << "MuscleFirstOrderActivationDynamicModel::"
                                "calcDerivative"
                             << endl;

                        cerr <<"    :" << getName().c_str() << endl;
                        cerr <<"    :Activation was outside of minActivation-1."
                             << endl;
                        cerr <<"    :Clamping activation to minActivation-1 "
                               "and continuing" 
                             << endl;
                        cerr << endl;
                    }*/

                }else{
                   SimTK_ERRCHK1_ALWAYS(false,
                    "MuscleFirstOrderActivationDynamicModel::calcDerivative",
                    "%s: calcDerivative is only valid for the 0th partial", 
                    getName().c_str());
                }
            }
            break;
        default:
            SimTK_ERRCHK1_ALWAYS(false,
            "MuscleFirstOrderActivationDynamicModel::calcDerivative",
            "%s: calcDerivative is only valid for the 0th and 1st derivative", 
            getName().c_str());
            
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

