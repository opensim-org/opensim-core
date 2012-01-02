#ifndef __JointReactionReference_h__
#define __JointReactionReference_h__
// JointReactionReference.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2010, Stanford University. All rights reserved.
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

#include "Reference.h"
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/Units.h>


namespace OpenSim {


class Units;

/*
class OSIMSIMULATION_API JointReactionWeight : public Object
{
private:
	PropertyDblArray _weightsProp;
	Array<double> &_weights;
	

public:
	JointReactionWeight() : Object(), _weights(_weightsProp.getValueDblArray()){};
	JointReactionWeight(std::string name, Array<double> forceWeights, Array<double> momentWeights) : Object(), 
		_weights(_weightsProp.getValueDblArray()),
		{_name = name; _weights = forceWeights; _weights = momentWeights;}

	//Copy
	JointReactionWeight(const JointReactionWeight &aWeight) : Object(aWeight), 
		_weights(_weightsProp.getValueDblArray()),
		{	
			_weights = aWeight._weights; 
		}


	//void setWeight(double weight) {_weight = weight; }
	void setWeights(Array<double> weightArray) {_weights = weightArray;}
	Array<double> getWeights() const {return _weights; };
	virtual Object* copy() const {
			JointReactionWeight *jWeight = new JointReactionWeight(*this);
			return(jWeight);
	}
}; // end of JointReactionWeight class
*/

//=============================================================================
//=============================================================================
/**
 * Reference values to be achieved for specified Markers that will be used
 * via optimization and/or tracking. Also contains a weighting that identifies
 * the relative importance of achieving one marker's reference relative to
 * another.
 *
 * @author Matt DeMers
 * @version 1.0
 */
class IKTaskSet;

class OSIMSIMULATION_API JointReactionReference : public Reference_<double>
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================

protected:

	/** Specify the reference joint reaction values from a file of columns in time. */
	//PropertyStr _markersFileProp;
	//std::string &_markersFile;

	/** Specify the individual weightings of markers to be matched.  */
	//PropertyObj _jointReactionWeightSetProp;
	//Set<JointReactionWeight> &_jointReactionWeightSet;

	/** Specify the default weight for markers not specified individually.  */
	//PropertyDbl _defaultWeightProp;
	//double &_defaultWeight;

	/** Property to indicate on or off state. */
	PropertyBool _onProp;
	// on or off state
	bool &_on;

	/** Specify the 3 weights dictating the importance of the 3 componenets of
	* the joint force.  */
	PropertyDblVec3 _forceWeightsProp;
	SimTK::Vec3 &_forceWeights;

	/** Specify the 3 weights dictating the importance of the 3 componenets of
	* the joint moment.  */
	PropertyDblVec3 _momentWeightsProp;
	SimTK::Vec3 &_momentWeights;

	/** A joint is an interface between two bodies, the child and the parent.  
	* this string determines which load is reported, the load on the child (default)
	* or the load on the parent. */
	PropertyStr _receivingBodyProp;
	std::string &_receivingBody;

	/** The joint load components can be expressed in one of three different reference frames 
	* (child, parent, ground) before being multiplied by the weights and added to the cost function.*/
	PropertyStr _referenceBodyFrameProp;
	std::string &_referenceBodyFrame;


private:
	// Implementation details

	// names of the individual components
	SimTK::Array_<std::string> _componentNames;

	// reference weights collected
	//SimTK::Array_<double> _weights;

	

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	virtual ~JointReactionReference() {};

	JointReactionReference();
	JointReactionReference(const JointReactionReference& aJointReactionReference);
	virtual Object* copy() const;

	//JointReactionReference(const std::string jointName, const SimTK::Array_<double> weights);

	JointReactionReference& operator=(const JointReactionReference &aRef) {
		Reference_<double>::operator=(aRef); 
		copyData(aRef); 
		return(*this); 
	}

	void copyData(const JointReactionReference &aRef){
												//_weights = aRef._weights;
												_forceWeights = aRef._forceWeights;
												_momentWeights = aRef._momentWeights;
												_receivingBody = aRef._receivingBody;
												_referenceBodyFrame = aRef._referenceBodyFrame;
												_on = aRef._on;}


	//--------------------------------------------------------------------------
	// Reference Interface
	//--------------------------------------------------------------------------
	/** Set wether the reference is currently on or off*/
	void setIsOn(bool aBool) { if(aBool) {_on = true; } else { _on = false; } };
	/** Get wether the reference is currently on or off*/
	bool getIsOn() { return _on; };
	/** Set the name of the joint to be optimized.*/
	//void setJointName(std::string aJointName) { _jointName = aJointName;};
	/** Chose which body of the joint pair (parent or child) the reference load 
	* should be associated with.*/
	void setReceivingBody(std::string aChildOrParent);
	std::string getReceivingBody() const { return _receivingBody; };
	/** Chose which bodies reference frame (parent, child, or ground) should be used
	* to express the components of the reference load.*/
	void setReferenceBodyFrame(std::string aChildParentOrGround);
	std::string getReferenceBodyFrame() const { return _referenceBodyFrame; };
	/* get the name of the reference components*/
	const SimTK::Array_<std::string>& getNames() const;
	void setNames(SimTK::Array_<std::string>& aNames);
	//std::string getJointName() const { return _jointName; };
	/** get the number of referettes (individual signals) in this Reference. All
	    return arrays are gauranteed to be this length */
	int getNumRefs() const { return 6; };

	/** get the value of the JointReactionReference three moment components first
	* followed by 3 force components.*/
	virtual void getValues(const SimTK::State &s, SimTK::Array_<double> &values) const;
	/** get the weighting (importance) of meeting this JointReactionReference in the same order as values*/
	virtual void getWeights(const SimTK::State &s, SimTK::Array_<double> &weights) const;
	/** get the weighting (importance) of meeting this JointReactionReference in independent vectors for forces and moments.*/
	virtual void getWeights(const SimTK::State &s, SimTK::Vec3 &momentWeights, SimTK::Vec3 &forceWeights) const;
	/** set the weighting (importance) of meeting this JointReactionReference in the same order as values*/
	void setWeights(SimTK::Array_<double> &weights);
	void setWeights(SimTK::Vec3 &momentWeights, SimTK::Vec3 &forceWeights);

	//--------------------------------------------------------------------------
	// Convenience Access
	//--------------------------------------------------------------------------
	//void setDefaultWeight(double weight) {_defaultWeight = weight; }	

private:
	// utility to define object properties including their tags, comments and
	// default values.
	void setupProperties();
	void setNull();
	
	

//=============================================================================
};	// END of class JointReactionReference
//=============================================================================
} // namespace

#endif // __JointReactionReference_h__
