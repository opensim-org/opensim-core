#ifndef __ControllerSet_h__
#define __ControllerSet_h__

// ControllerSet.h
// Author: Frank C. Anderson, Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>
#include "SimTKsimbody.h"

namespace OpenSim {

class Model;
class Storage;

//=============================================================================
//=============================================================================
/**
 * A class for holding and managing a set of controllers for a model.
 *
 * @authors Jack Middleton, Ajay Seth 
 * @version 2.0
 */

//=============================================================================
class OSIMSIMULATION_API ControllerSet : public ModelComponentSet<Controller> {
OpenSim_DECLARE_CONCRETE_OBJECT(ControllerSet, ModelComponentSet<Controller>);

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ControllerSet();
	ControllerSet(Model& model);
	ControllerSet(const ControllerSet &aControllerSet);
	ControllerSet(Model& model, const std::string &aFileName,  bool aUpdateFromXMLNode = true);
	virtual ~ControllerSet();

	void copyData(const ControllerSet &aAbsControllerSet);
private:
	void setNull();
    void setupSerializedMembers();

    /**
     *   storage object containing the storage object
     */
     Storage* _controlStore;

    /**
     *   set of actuators controlled by the set of controllers 
     */
     Set<Actuator>* _actuatorSet;


	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	ControllerSet& operator=(const ControllerSet &aSet);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:

	bool set(int aIndex, Controller *aController);
    bool addController(Controller *aController);


    virtual void constructStorage();
    virtual void storeControls( const SimTK::State& s, int step );
    virtual void printControlStorage( const std::string& fileName) const;
    virtual void setActuators( Set<Actuator>& );

    virtual bool check() const;

    virtual void setDesiredStates( Storage* yStore); 

	// Controller interface
	virtual void computeControls(const SimTK::State& s, SimTK::Vector &controls) const; 

    virtual void printInfo() const;
//=============================================================================
};	// END of class ControllerSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim


#endif // __ControllerSet_h__


