/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ControllerSet.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Peter Loan                                   *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


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
namespace OpenSim {
template class OSIMSIMULATION_API ModelComponentSet<Controller>;
}
#endif


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */

ControllerSet::ControllerSet(Model& model) : ModelComponentSet<Controller>(model)
{
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
    if(aUpdateFromXMLNode) updateFromXMLDocument();
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aControllerSet ControllerSet to be copied.
 */
ControllerSet::ControllerSet(const ControllerSet& aControllerSet) :
    ModelComponentSet<Controller>(aControllerSet)
{

    // Class Members
    copyData(aControllerSet);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy the member variables of the ControllerSet.
 *
 * @param aControllerSet controller set to be copied
 */
void ControllerSet::copyData(const ControllerSet& aControllerSet)
{
    _actuatorSet =  aControllerSet._actuatorSet;
    const Storage* source = aControllerSet._controlStore.get();
    if (source) {
        _controlStore.reset(new Storage(*source, true));
    } else {
        _controlStore.reset();
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

void ControllerSet::constructStorage() 
{
    Array<string> columnLabels;

    // CONTROLS
    _controlStore.reset(new Storage(1023,"controls"));
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
        _controlStore->store( step, s.getTime(), getModel().getNumControls(), 
                              &(getModel().getControls(s)[0]) );
    }
}

// write out the controls to disk
void ControllerSet::printControlStorage( const string& fileName)  const
{
   _controlStore->print(fileName);
}

TimeSeriesTable ControllerSet::getControlTable() const {
    return _controlStore->getAsTimeSeriesTable();
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
       if( get(i).isEnabled() ) {
           TrackingController *controller =
               dynamic_cast<TrackingController *>(&get(i));
           if(controller != NULL)
                controller->setDesiredStatesStorage( yStore );
       }
   }
}

void ControllerSet::printInfo() const 
{
    std::cout << " Number of controllers = " << getSize() << std::endl;

    for(int i=0;i<getSize(); i++ ) {
      Controller& c = get(i);
      if( c.isEnabled() ) {
          printf(" controller %d =%llx %s model=%llx \n", 
              i+1, (unsigned long long)&c, c.getName().c_str(), 
              (unsigned long long)&c.getModel() );

          const Set<const Actuator>& actSet = c.getActuatorSet();
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


void ControllerSet::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
{
    std::cout << "IN ControllerSet::computeControls()" << std::endl;
    /**
    for(int i=0;i<getSize(); i++ ) {
        if(!get(i).isDisabled() )
            get(i).computeControls(s, controls);
    }*/
}
