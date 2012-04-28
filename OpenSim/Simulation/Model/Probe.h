#ifndef OPENSIM_PROBE_H_
#define OPENSIM_PROBE_H_

// Probe.h
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

// INCLUDE
#include "Model.h"
#include "ModelComponent.h"

namespace OpenSim {

class Model;

//==============================================================================
//==============================================================================
/**
 * This class represents a Probe which will can perform an "operation" on any model value.
 * Each subclass represents a different type of Probe. The actual "operation" is done by a
 * SimTK::Measure. In creating child class Probes, the user must override the
 * computeProbeValue() method.
 *
 * @author Tim Dorn
 */
class OSIMSIMULATION_API Probe : public ModelComponent
{
	OpenSim_DECLARE_ABSTRACT_OBJECT(Probe, ModelComponent);
//=============================================================================
// DATA
//=============================================================================
protected:
	SimTK::Measure afterOperationValue;



//=============================================================================
// METHODS
//=============================================================================
public:
	
	Probe();
	Probe(const Probe &aProbe);
	virtual ~Probe();

	/**
	 * deserialization from XML, necessary so that derived classes can (de)serialize
	 */
	//Probe(SimTK::Xml::Element& aNode): ModelComponent(aNode) {setNull(); setupProperties(); };

#ifndef SWIG
	Probe& operator=(const Probe &aProbe);
#endif

	void copyData(const Probe &aProbe);
		

	/** Returns trus if the Probe is disabled or false if the probe is enabled. */
	virtual bool isDisabled() const;
	/** Set the Probe as disabled (true) or enabled (false). */
	virtual void setDisabled(bool isDisabled);

	/** Return the operation being performed on the probe value. */
	virtual std::string getOperation() const;
	/** Set the operation being performed on the probe value. */
	virtual void setOperation(std::string operation);

	/** Return the operation parameter for the operation. */
	virtual double getOperationParameter() const;
	/** Set the operation parameter for the operation. */
	virtual void setOperationParameter(double operation_parameter);


protected:
	virtual void setup(Model& model);
	virtual void createSystem(SimTK::MultibodySystem& system) const;
    virtual void setDefaultsFromState(const SimTK::State& state);


public:

	/**Computes the probe value (this is the value of the probe prior to any operation being performed on it.
	   Probe value is computed at the SimTK Report Stage
	   This method must be overridden for each subclass Probe.

       @param state System state   
       @return		The SimTK::Vector of probe values
    */
	virtual SimTK::Vector computeProbeValue(const SimTK::State& state) const=0;

	/**Returns the column labels for the probe values for reporting. */
	virtual Array<std::string> getRecordLabels() const;

	/**Returns the probe values after being operated on. */
	virtual Array<double> getRecordValues(const SimTK::State& state) const;


private:

	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class Probe
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_PROBE_H_


