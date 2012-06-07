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
ForceProbe::ForceProbe(const Array<string>& force_names)
{
    setNull();
    constructProperties();

    set_force_names(force_names);
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
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the Force names that the ForceProbe is acting on.
 */
const Property<string>& ForceProbe::getForceNames() const
{
    return getProperty_force_names();
}

//_____________________________________________________________________________
/**
 * Set the Force names that the ForceProbe is acting on.
 */
void ForceProbe::setForceNames(const Array<string>& forceNames)
{
    set_force_names(forceNames);
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
 * Compute the Force upon which the Probe operation will be based on.
 */
double ForceProbe::computeProbeValue(const State& s) const
{
    int nF = getForceNames().size();
    double TotalF = 0;		// Initialize at zero

    // Loop through each force in the list of force_names
    for (int i=0; i<nF; i++)
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

        // Append to total "Force" force
        TotalF += Ftmp;
    }

    return TotalF;
}
