// ControllerSet.cpp
// Authors: Frank C. Anderson, Peter Loan
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include "ControllerSet.h"
#include "Model.h"
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Control/TrackingController.h>
#include "Actuator.h" 
#include <OpenSim/Common/Set.h>
#include "SimTKsimbody.h"

using namespace std;
using namespace OpenSim;

#ifndef SWIG
template class OSIMSIMULATION_API ModelComponentSet<Controller>;
#endif


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ControllerSet::~ControllerSet()
{
	delete _controlStore;
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ControllerSet::ControllerSet()
{
	setNull();
}
ControllerSet::ControllerSet(Model& model) : ModelComponentSet<Controller>(model)
{
   setNull();
}
//_____________________________________________________________________________
/**
 * Construct a controller set from file.
 *
 * @param aFileName Name of the file.
 */

ControllerSet::ControllerSet(Model& model, const std::string &aFileName, bool aUpdateFromXMLNode) :
     ModelComponentSet<Controller>(model, aFileName, false)
{
    setNull();
    if(aUpdateFromXMLNode) updateFromXMLDocument();
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aControllerSet ControllerSet to be copied.
 */
ControllerSet::ControllerSet(const ControllerSet &aControllerSet) :
	ModelComponentSet<Controller>(aControllerSet)
{
	setNull();

	// Class Members
	copyData(aControllerSet);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this ControllerSet to their null values.
 */
void ControllerSet::setNull()
{
	// TYPE
	setType("ControllerSet");
    _actuatorSet = NULL;
    _controlStore = NULL;

   // PROPERTIES
   setupSerializedMembers();
   
   // NAME
   setName("ControllerSet");


}

//_____________________________________________________________________________
/**
 * Copy this ControllerSet and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this ControllerSet.
 */
Object* ControllerSet::copy() const
{
	ControllerSet *conSet = new ControllerSet(*this);
	return(conSet);
}

//_____________________________________________________________________________
/**
 * Copy the member variables of the ControllerSet.
 *
 * @param aControllerSet controller set to be copied
 */
void ControllerSet::copyData(const ControllerSet &aControllerSet)
{
	_actuatorSet =  aControllerSet._actuatorSet;
	const Storage *source = (Storage *)aControllerSet._controlStore;
	if(source == NULL){
		_controlStore =  NULL;
	} else {
		_controlStore =  new Storage(*source, true);
	}
}



//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
ControllerSet& ControllerSet::operator=(const ControllerSet &aControllerSet)
{
	// BASE CLASS
	Set<Controller>::operator=(aControllerSet);

	// Class Members
	copyData(aControllerSet);

	return(*this);
}


//_____________________________________________________________________________
/**
 * Add a Controller  to the set.  A copy of the specified controller
 * is not made.
 *
 * This method overrides the method in Set<Controller> so that several
 * internal variables of the controller set can be updated.
 *
 * @param aController Pointer to the controller to be appended.
 * @return True if successful; false otherwise.
 */
bool ControllerSet::addController(Controller *aController)
{
	bool success = Set<Controller>::append(aController);

	if(success) {
		aController->setup(*_model);
	}

	return(success);
}
//_____________________________________________________________________________
/**
 * Set the controller at an index.  A copy of the specified controller is NOT made.
 * The controller previously set a the index is removed (and deleted).
 *
 * This method overrides the method in Set<Controller> so that several
 * internal variables of the controller set can be updated.
 *
 * @param aIndex Array index where the controller is to be stored.  aIndex
 * should be in the range 0 <= aIndex <= getSize();
 * @param acontroller Pointer to the controller to be set.
 * @return True if successful; false otherwise.
 */
bool ControllerSet::set(int aIndex,Controller *aController)
{
	bool success = Set<Controller>::set(aIndex,aController);

	return(success);
}

//=============================================================================
// CHECK
//=============================================================================
//_____________________________________________________________________________
/**
 * Check that all actuators have a valid controller.
 */
bool ControllerSet::check() const
{
	bool status=true;
	return(status);

}

void ControllerSet::constructStorage() 
{
    Array<string> columnLabels;

    // CONTROLS
	delete _controlStore;
    _controlStore = new Storage(512,"controls");
    columnLabels.append("time");

    for(int i=0;i<_actuatorSet->getSize();i++)
		columnLabels.append(_actuatorSet->get(i).getName());

    _controlStore->setColumnLabels(columnLabels);
        
}

void ControllerSet::storeControls( const SimTK::State& s, int step  )
{
    int size = _actuatorSet->getSize();
    
	if( size > 0 )
	{
		SimTK::Vector controls(_actuatorSet->getSize());
	    
		for (int i = 0; i < _actuatorSet->getSize(); i++) {
			controls[i] = _actuatorSet->get(i).getControl(s);
		} 

		_controlStore->store( step, s.getTime(), _actuatorSet->getSize(), &controls[0] );
    }
}

// write out the controls to disk
void ControllerSet::printControlStorage( const string& fileName)  const
{
   _controlStore->print(fileName);
}

void ControllerSet::setActuators( Set<Actuator>& as) 
{
    _actuatorSet = &as;
    constructStorage();
}

// get the initial time for all the controllers 

void ControllerSet::setDesiredStates( Storage* yStore)
{
   for(int i=0;i<getSize();i++ ) {
       if( !get(i).isDisabled() ) {
		   TrackingController *controller = dynamic_cast<TrackingController *>(&get(i));
		   if(controller != NULL)
				controller->setDesiredStatesStorage( yStore );
       }
   }
}

// post deserialize initialization

void ControllerSet::setup(Model& aModel)
{
	// BASE CLASS
	ModelComponentSet<Controller>::setup(aModel);
}

void ControllerSet::printInfo() const 
{
    std::cout << " Number of controllers = " << getSize() << std::endl;

    for(int i=0;i<getSize(); i++ ) {
      Controller& c = get(i);
      if( !c.isDisabled() ) {
          printf(" controller %d =%llx %s model=%llx \n", 
              i+1, (unsigned long long)&c, c.getName().c_str(), 
              (unsigned long long)&c.getModel() );

          const Set<Actuator>& actSet = c.getActuatorSet();
          if( actSet.getSize() > 0 ) {
               std::cout << "Actuators" << std::endl;
               for(int j=0;j<get(i).getActuatorSet().getSize(); j++ ) {
                   std::cout <<get(i).getActuatorSet().get(j).getName() << std::endl;
               }
           }
      } else { 
         printf(" controller %d =%llx %s model=%llx DISABLED \n", 
             i+1, (unsigned long long)&c, c.getName().c_str(), 
             (unsigned long long)&c.getModel() );
      }
   }
}

void ControllerSet::createSystem(SimTK::MultibodySystem& system)  
{
   for(int i=0;i<getSize(); i++ ) {
       //get(i).setModel(*_model);
       if(!get(i).isDisabled() )
		   get(i).createSystem(system);
   }
}

void ControllerSet::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
{
	for(int i=0;i<getSize(); i++ ) {
		if(!get(i).isDisabled() )
			get(i).computeControls(s, controls);
	}
}

//_____________________________________________________________________________
/**
 * Set up the serialized member variables.
 */
void ControllerSet::setupSerializedMembers()
  {
  }
