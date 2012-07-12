// ForceProbe.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Tim Dorn
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2012 Stanford University
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


//=============================================================================
// INCLUDE
//=============================================================================
#include "ForceProbe.h"
#include "ForceSet.h"


using namespace std;
using namespace SimTK;
using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S) AND SETUP
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, and copy
// assignment operator.


//_____________________________________________________________________________
/**
 * Default constructor.
 */
ForceProbe::ForceProbe()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/** 
 * Convenience constructor
 */
ForceProbe::ForceProbe(const Array<string>& force_names, 
    const bool sum_forces_together, const double exponent)
{
    setNull();
    constructProperties();

    set_force_names(force_names);
    set_sum_forces_together(sum_forces_together);
    set_exponent(exponent);
}


//_____________________________________________________________________________
// Set the data members of this ForceProbe to their null values.
void ForceProbe::setNull()
{
    // no data members
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ForceProbe::constructProperties()
{
    constructProperty_force_names();
    constructProperty_sum_forces_together(false);
    constructProperty_exponent(1.0);
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Returns the name(s) of the Forces being probed.
 */
const Property<string>& ForceProbe::getForceNames() const
{
    return getProperty_force_names();
}

//_____________________________________________________________________________
/**
 * Returns whether to report sum of all forces together
   or report the forces individually.
 */
const bool ForceProbe::getSumForcesTogether() const
{
    return get_sum_forces_together();
}

//_____________________________________________________________________________
/**
 * Returns the exponent to apply to each force.
 */
const double ForceProbe::getExponent() const
{
    return get_exponent();
}

//_____________________________________________________________________________
/**
 * Sets the name(s) of the Forces being probed.
 */
void ForceProbe::setForceNames(const Array<string>& forceNames)
{
    set_force_names(forceNames);
}

//_____________________________________________________________________________
/**
 * Sets whether to report sum of all force values together
   or report the force values individually.
 */
void ForceProbe::setSumForcesTogether(const bool sum_forces_together)
{
    set_sum_forces_together(sum_forces_together);
}

//_____________________________________________________________________________
/**
 * Sets the exponent to apply to each force.
 */
void ForceProbe::setExponent(const double exponent)
{
    set_exponent(exponent);
}



//=============================================================================
// MODEL COMPONENT METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this ForceProbe.
 */
void ForceProbe::connectToModel(Model& model)
{
    Super::connectToModel(model);

    // check that each Force in the force_names array exists in the model
    int nF = getForceNames().size();
    for (int i=0; i<nF; i++) {
        string forceName = getForceNames()[i];
        int k = model.getForceSet().getIndex(forceName);
        if (k<0) {
            string errorMessage = getConcreteClassName() + ": Invalid Force '" 
                                  + forceName + "' specified in <force_names>.";
            throw OpenSim::Exception(errorMessage);
        }
    }
}



//=============================================================================
// COMPUTATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the Force.
 */
SimTK::Vector ForceProbe::computeProbeInputs(const State& s) const
{
    int nF = getForceNames().size();
    SimTK::Vector TotalF;

    if (getSumForcesTogether()) {
        TotalF.resize(1);
        TotalF(0) = 0;       // Initialize to zero
    }
    else
        TotalF.resize(nF);

    // Loop through each force in the list of force_names
    for (int i=0; i<nF; ++i)
    {
        double Ftmp = 0.0;
        string forceName = getForceNames()[i];
        int k = _model->getForceSet().getIndex(forceName);

        // Get the "Force" force from the Force object method getRecordValues(s)
        Array<double> forceValues = _model->getForceSet().get(k).getRecordValues(s);
        
        // For body forces (which have 6 output forces), we give a warning
        if(forceValues.getSize() != 1) {
            cout << "Warning: Force [" << forceName << "] does not have a single output (it has " << forceValues.getSize() << "). Summing together.." << endl;
            for (int j=0; j<forceValues.getSize(); j++)
                Ftmp += forceValues.get(j);
        }
        else 
            Ftmp = forceValues.get(0);

        // Append to output vector
        if (getSumForcesTogether())
            TotalF(0) += std::pow(Ftmp, getExponent());
        else
            TotalF(i) = std::pow(Ftmp, getExponent());
    }

    return TotalF;
}


//_____________________________________________________________________________
/** 
 * Provide labels for the probe values being reported.
 */
Array<string> ForceProbe::getProbeLabels() const 
{
    Array<string> labels;

    // Report sum of force values
    if (getSumForcesTogether()) {
        if (getScaleFactor() != 1.0) {
            char n[10];
            sprintf(n, "%f", getScaleFactor());
            labels.append(getName()+"_Summed_SCALED_BY_"+n+"X");
        }
        else
            labels.append(getName()+"_Summed_"+getOperation());
    }

    // Report force values individually
    else {
        for (int i=0; i<getForceNames().size(); ++i) {
            if (getScaleFactor() != 1.0) {
            char n[10];
            sprintf(n, "%f", getScaleFactor());
            labels.append(getName()+"_"+getForceNames()[i]+"_SCALED_BY_"+n+"X");
        }
        else
            labels.append(getName()+"_"+getForceNames()[i]+"_"+getOperation());
        }
    }


    return labels;
}