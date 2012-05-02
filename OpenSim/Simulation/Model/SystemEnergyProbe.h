#ifndef OPENSIM_SYSTEM_ENERGY_PROBE_H_
#define OPENSIM_SYSTEM_ENERGY_PROBE_H_

// SystemEnergyProbe.h
// Author: Tim Dorn
/*
 * Copyright (c)  2011, Stanford University. All rights reserved. 
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


// INCLUDE
#include "Probe.h"
#include "Model.h"


namespace OpenSim {

class Model;

//==============================================================================
//==============================================================================
/**
 * SystemEnergyProbe is a ModelComponent Probe for computing an operation on a 
 * total system energy during a simulation.
 * E.g. Work is the integral of power with respect to time.
 *
 * @author Tim Dorn
 * @version 1.0
 */
class OSIMSIMULATION_API SystemEnergyProbe : public Probe  
{
    OpenSim_DECLARE_CONCRETE_OBJECT(SystemEnergyProbe, Probe);
//=============================================================================
// DATA
//=============================================================================
protected:

//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // Constructor(s) and Setup
    //--------------------------------------------------------------------------
    /** Default constructor */
    SystemEnergyProbe();
    /** Convenience constructor */
    SystemEnergyProbe(bool computeKE, bool computePE);
    /** Copy constructor */
    SystemEnergyProbe(const SystemEnergyProbe &aSystemEnergyProbe);
    /** Destructor */
    virtual ~SystemEnergyProbe();
    void copyData(const SystemEnergyProbe &aProbe);

private:
    void setNull();
    void setupProperties();

public:

    //--------------------------------------------------------------------------
    // Operators
    //--------------------------------------------------------------------------
#ifndef SWIG
    SystemEnergyProbe& operator=(const SystemEnergyProbe &aSystemEnergyProbe);
#endif


    //--------------------------------------------------------------------------
    // Get and Set
    //--------------------------------------------------------------------------
    /** Returns whether kinetic energy is to be included in the system energy computation. */
    bool getComputeKineticEnergy() const;

    /** Returns whether kinetic energy is to be included in the system energy computation. */
    bool getComputePotentialEnergy() const;

    /** Sets whether kinetic energy is to be included in the system energy computation. */
    void setComputeKineticEnergy(bool c);

    /** Sets whether potential energy is to be included in the system energy computation. */
    void setComputePotentialEnergy(bool c);


    //-----------------------------------------------------------------------------
    // Computation
    //-----------------------------------------------------------------------------
    /** Compute the System energy which the Probe operation will be based on. */
    virtual double computeProbeValue(const SimTK::State& state) const;


    //--------------------------------------------------------------------------
    // ModelComponent Interface
    //--------------------------------------------------------------------------
protected:
    virtual void setup(Model& aModel);
    

//=============================================================================
};	// END of class SystemEnergyProbe
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_SYSTEM_ENERGY_PROBE_H_


