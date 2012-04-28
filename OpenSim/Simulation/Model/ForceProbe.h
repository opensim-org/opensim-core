#ifndef OPENSIM_FORCE_PROBE_H_
#define OPENSIM_FORCE_PROBE_H_

// ForceProbe.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Tim Dorn
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2012, Stanford University. All rights reserved. 
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

#include "Probe.h"
#include "Model.h"
#include "ForceSet.h"

namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 * ForceProbe is a ModelComponent Probe for computing an operation on a 
 * force or sum of forces in the model during a simulation.
 * E.g. Impulse is the integral of force with respect to time.
 *
 * @author Tim Dorn
 */

class Model;
class Force;

class OSIMSIMULATION_API ForceProbe : public Probe 
{
	OpenSim_DECLARE_CONCRETE_OBJECT(ForceProbe, Probe);
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
	ForceProbe();
	/** Convenience constructor */
	ForceProbe(Array<std::string> force_names);
	/** Copy constructor */
	ForceProbe(const ForceProbe &aForceProbe);
	/** Destructor */
	virtual ~ForceProbe();
	void copyData(const ForceProbe &aProbe);
	
private:
	void setNull();
	void setupProperties();

public:

	//--------------------------------------------------------------------------
	// Operators
	//--------------------------------------------------------------------------
#ifndef SWIG
	ForceProbe& operator=(const ForceProbe &aObject);
#endif


	//--------------------------------------------------------------------------
	// Get and Set
	//--------------------------------------------------------------------------
	/** Returns the name(s) of the Forces being probed */
	const Property<std::string>& getForceNames() const;

	/** Sets the name(s) of the Forces being probed */
	void setForceNames(const Array<std::string>& aForceNames);


	//-----------------------------------------------------------------------------
	// Computation
	//-----------------------------------------------------------------------------
	/** Compute the Force upon which the Probe operation will be based on. */
	virtual SimTK::Vector computeProbeValue(const SimTK::State& state) const;


	//--------------------------------------------------------------------------
	// ModelComponent Interface
	//--------------------------------------------------------------------------
protected:
	virtual void setup(Model& aModel);


//=============================================================================
};	// END of class ForceProbe

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef OPENSIM_FORCE_PROBE_H_
