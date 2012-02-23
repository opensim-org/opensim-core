#ifndef __ForceSet_h__
#define __ForceSet_h__

// ForceSet.h
// Author: Ajay Seth, Jack Middleton 
/*
 * Copyright (c)  2009, Stanford University. All rights reserved. 
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


// INCLUDES
#include "Force.h"
#include "Muscle.h"
#include "ModelComponentSet.h"

namespace OpenSim {

class Model;
class Actuator;
class Muscle;

//=============================================================================
//=============================================================================
/**
 * A class for holding and managing a set of forces for a model.
 * This class is based on ModelComponentSet
 *
 * @authors Ajay Seth, Jack Middleton 
 * @version 1.0
 */

//=============================================================================
class OSIMSIMULATION_API ForceSet : public ModelComponentSet<Force>
{

//=============================================================================
// DATA
//=============================================================================
protected:

   /** The subset of Forces that are Actuators. */
    Set<Actuator> _actuators;

	/** The subset of Forces that are Muscles. */
	Set<Muscle> _muscles;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ForceSet();
	ForceSet(Model& model);
	ForceSet(Model& model, const std::string &aFileName, bool aUpdateFromXMLNode = true);
	ForceSet(const ForceSet &aForceSet);
	virtual ~ForceSet();
	virtual Object* copy() const;
	void copyData(const ForceSet &aAbsForceSet);

private:
	void setNull();
	void setupSerializedMembers();
    void updateActuators();
	void updateMuscles();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	ForceSet& operator=(const ForceSet &aSet);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	virtual void setup(Model& aModel);

	// FORCE
	bool remove(int aIndex);
	bool append(Force *aForce);
#ifndef SWIG
	bool append(Force &aForce);
#endif
	bool append(ForceSet &aForceSet, bool aAllowDuplicateNames=false);
	bool set(int aIndex, Force *aForce);
    bool insert(int aIndex, Force *aObject);

    // subsets 
    const Set<Actuator>& getActuators() const;
    Set<Actuator>& updActuators();
	const Set<Muscle>& getMuscles() const;
    Set<Muscle>& updMuscles();

    // STATES
    void getStateVariableNames(Array<std::string> &rNames) const;


	//--------------------------------------------------------------------------
	// CHECK
	//--------------------------------------------------------------------------
	bool check() const;

//=============================================================================
};	// END of class ForceSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim


#endif // __ForceSet_h__


