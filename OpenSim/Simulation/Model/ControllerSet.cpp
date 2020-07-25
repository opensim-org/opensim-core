/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ControllerSet.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2018 Stanford University and the Authors                *
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

using namespace std;
using namespace OpenSim;

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
    return _controlStore->exportToTable();
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
    log_cout(" Number of controllers = {}");

    for(int i=0;i<getSize(); i++ ) {
      Controller& c = get(i);
      if( c.isEnabled() ) {
          log_cout(" controller {} ={} {} model={}", 
              i+1, (unsigned long long)&c, c.getName().c_str(), 
              (unsigned long long)&c.getModel() );

          const Set<const Actuator>& actSet = c.getActuatorSet();
          if( actSet.getSize() > 0 ) {
               log_cout("Actuators");
               for(int j=0;j<get(i).getActuatorSet().getSize(); j++ ) {
                   log_cout("{}", get(i).getActuatorSet().get(j).getName());
               }
           }
      } else { 
         log_cout(" controller {} ={} {} model={} DISABLED", 
             i+1, (unsigned long long)&c, c.getName().c_str(), 
             (unsigned long long)&c.getModel() );
      }
   }
}


void ControllerSet::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
{
    log_trace("IN ControllerSet::computeControls()");
    /**
    for(int i=0;i<getSize(); i++ ) {
        if(!get(i).isDisabled() )
            get(i).computeControls(s, controls);
    }*/
}
