/* JointReactionReference.cpp 
* Author: Matt DeMers
* Copyright (c)  2010 Stanford University
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

#include "JointReactionReference.h"
#include <OpenSim/Common/Units.h>

using namespace std;
using namespace SimTK;

namespace OpenSim {

//______________________________________________________________________________
/**
 * An implementation of the JointReactionReference 
 *
 * @param
 */
JointReactionReference::JointReactionReference() : Reference_<double>(),
	_receivingBody(_receivingBodyProp.getValueStr()),
	_referenceBodyFrame(_referenceBodyFrameProp.getValueStr()),
	_on(_onProp.getValueBool()),
	_forceWeights(_forceWeightsProp.getValueDblVec()),
	_momentWeights(_momentWeightsProp.getValueDblVec())
{
	setupProperties();
	setNull();	

}
// copy constructor
JointReactionReference::JointReactionReference(const JointReactionReference& aJointReactionReference) : Reference_<double>(),
	_receivingBody(_receivingBodyProp.getValueStr()),
	_referenceBodyFrame(_referenceBodyFrameProp.getValueStr()),
	_on(_onProp.getValueBool()),
	_forceWeights(_forceWeightsProp.getValueDblVec()),
	_momentWeights(_momentWeightsProp.getValueDblVec())
{
	setName(aJointReactionReference.getName());
	setupProperties();
	setNull();
	copyData(aJointReactionReference);
}

// Utility to set a default state to all members before serializing
void JointReactionReference::setNull()
{
	//setName("JointReactionReference");
	
	_momentWeights = Vec3(1.0, 1.0, 1.0);
	_forceWeights = Vec3(1.0, 1.0, 1.0);
	//_weights.assign(6, 1.0); //make it all ones
	_on = true;
	_receivingBody = "child";
	_referenceBodyFrame = "parent";
	_componentNames.resize(6);

	_componentNames[0] = "mx";
	_componentNames[1] = "my";
	_componentNames[2] = "mz";
	_componentNames[3] = "fx";
	_componentNames[4] = "fy";
	_componentNames[5] = "fz";

}
// utility to define object properties including their tags, comments and 
// default values.
void JointReactionReference::setupProperties()
{
	Array<double> ones(1.0, 3);

	_onProp.setComment("Flag (true or false) indicating whether or not a task is enabled.");
	_onProp.setName("on");
	//_onProp.setValue(true);
	_propertySet.append(&_onProp);

	_forceWeightsProp.setComment("Array of 3 positive scalars indicating the importance of each x, y, z component"
		"of the joint force.");
	_forceWeightsProp.setName("force_weights");
	//_forceWeightsProp.setValue(ones);
	_propertySet.append( &_forceWeightsProp );

	_momentWeightsProp.setComment("Array of 3 positive scalars indicating the importance of each x, y, z component"
		"of the joint moment.");
	_momentWeightsProp.setName("moment_weights");
	//_momentWeightsProp.setValue(ones);
	_propertySet.append( &_momentWeightsProp );

	_receivingBodyProp.setComment("A joint is an interface between two bodies, the child and the parent.  This string "
		"determines which load is reported, the load applied to the child (default) or the load applied to the parent.");
	_receivingBodyProp.setName("receiving_body");
	//_receivingBodyProp.setValue("child");
	_propertySet.append( &_receivingBodyProp );

	_referenceBodyFrameProp.setComment("The joint load components can be expressed in one of three different reference"
		"frames (child, parent, ground) before being multiplied by the weights and added to the cost function.");
	_referenceBodyFrameProp.setName("reference_frame_body");
	//_referenceBodyFrameProp.setValue("global");
	_propertySet.append( &_referenceBodyFrameProp );

	//_jointName = this->getName();

}

/**
* pick which body of the joint pair (parent or child) the reference load should be associated with
* 
* @param aChildOrParent: string that equals "child" or "parent", specifying to report loads occuring
*							on the child or parent body respectively.
*/
void JointReactionReference::setReceivingBody(std::string aChildOrParent){
	// convert to lower case
	std::transform(aChildOrParent.begin(),aChildOrParent.end(),aChildOrParent.begin(), ::tolower);
	if(aChildOrParent=="child") { _receivingBody = "child"; }
	else if(aChildOrParent=="parent") { _receivingBody = "parent"; }
	else {
		std::cout << "Must chose 'child' or 'parent' body for the " << getName() << "JointReactionReference."<< std::endl;
		std::cout << "... setting body to default (child)." << std::endl;
	}
}

/**
* pick body frame (parent or child or ground) the reference load should be expressed in
* 
* @param aChildParentOrGround: string that equals "child", "parent", or "ground", specifying to use loads expressed
*							in the child, parent, or ground body's reference frame respectively.
*/
void JointReactionReference::setReferenceBodyFrame(std::string aChildParentOrGround){
	// convert to lower case
	std::transform(aChildParentOrGround.begin(),aChildParentOrGround.end(),aChildParentOrGround.begin(), ::tolower);
	if(aChildParentOrGround=="child") { _referenceBodyFrame = "child"; }
	else if(aChildParentOrGround=="parent") { _referenceBodyFrame = "parent"; }
	else if(aChildParentOrGround=="ground") { _referenceBodyFrame = "ground"; }
	else {
		std::cout << "Must chose 'child', 'parent', or 'ground' frame for the " << getName() << "JointReactionReference."<< std::endl;
		std::cout << "... setting frame to default (ground)." << std::endl;
	}
}
/**
 * get the names of the joint reaction reference components
 */
const SimTK::Array_<std::string>& JointReactionReference::getNames() const {
	return _componentNames;
}

/**
 * manually set the names of the joint reaction components
 *
 *@param aNames: SimTK::Array_ of six strings to be used as names for the joint load components.
 */
void JointReactionReference::setNames(SimTK::Array_<std::string>& aNames) {
	if( aNames.size() == getNumRefs()) { _componentNames = aNames; }
	else { std::cout << "You can only specify 6 names for a JointReactionReference because it only has 6 components." << std::endl; }

}
//______________________________________________________________________________
/**
 * get the values of the JointReactionReference 
 *
 * @param s:  SimTK state
 * @param values: predefined SimTK::Array where the target values of the reference will be returned.
 */
void  JointReactionReference::getValues(const SimTK::State &s, SimTK::Array_<double> &values) const
{
	double time =  s.getTime();

	for(int i=0; i<6; i++) {
		values[i] = 0;
	}
}


/** 
* get the 6 weights of the load components 
* 
* @param s:  SimTK state
* @param weights: predefined SimTK::Array where the target weights of the reference will be returned.
*/
void  JointReactionReference::getWeights(const SimTK::State &s, SimTK::Array_<double> &weights) const
{
	for(int i=0; i<3; i++){
		weights[i] = _momentWeights[i];
		weights[i+3] = _forceWeights[i];
	}
}

/** get the weighting (importance) of meeting this JointReactionReference in independent vectors for forces and moments.
* 
* @param: s: SimtK state
* @param: momentWeights: predefined SimTK::Vec3 that will be filled with the weights for moment components.
* @param: forceWeihts: predefined SimTK::Vec3 that will be filled with the weights of the force components
*/
void JointReactionReference::getWeights(const SimTK::State &s, SimTK::Vec3 &momentWeights, SimTK::Vec3 &forceWeights) const{
	momentWeights = _momentWeights;
	forceWeights = _forceWeights;
}

/**
* set the 6 weights for the components of the joint load from a simtk array of doubles.
* 
* @param weights: predifined SimTK::Array that specifies the six weights applied to
*					the joint load terms.  First 3 are weights for the moment x, y z
*					components.  Last 3 are the weighs for the force x, y , z compoents.
*/
void  JointReactionReference::setWeights(SimTK::Array_<double> &weights){
	// copy to SimTK::Array of weights used by default reference interface
	//_weights = weights;
	
	// copy to the convenience SimTK::Vec3 memebers
	for(int i=0; i<3; i++){
		_momentWeights[i] = weights[i];
		_forceWeights[i] = weights[i+3];
	}
}

/** 
* set the 6 weights for the components of the joint load from opensim arrays specifying the 
* moment and force weights independently.
* 
* @param momentWeights:  SimTK::Array of the 3 weights applied to the joint moment components
* @param forceWeights:  SimTK::Array of the 3 weights applied to the joint force components
*/
void  JointReactionReference::setWeights(SimTK::Vec3 &momentWeights, SimTK::Vec3 &forceWeights){
	// copy to SImTK::Array of weights used by default reference interface
	/*
	for(int i=0; i<3; i++){
		_weights[i] = momentWeights[i];
		_weights[i+3] = forceWeights[i];
	}
	*/

	// copy to convenience SimTK::Vec3 members
	_momentWeights = momentWeights;
	_forceWeights = forceWeights;
}



} // end of namespace OpenSim

