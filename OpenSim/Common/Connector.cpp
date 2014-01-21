/* -------------------------------------------------------------------------- *
 *                            OpenSim: Connector.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Michael Sherman                                      *
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

#include "Connector.h"

using namespace SimTK;

namespace OpenSim {

Connector::Connector()
{

}

Connector::Connector( const std::string& type, const SimTK::Stage& connectAtStage )
{

}


bool Connector::isConnected() const
{
	return !_source.empty();
}

void Connector::disconnect()
{
	_source.clear();
}

void Connector::connect( const Object* object )
{
	// Check that object being connected is of compatible type
	if(object->getConcreteClassName() ==  _type){
		_source = object;
	}else{
		 // disconnect if we had another object connected
		_source.empty();
		std::string msg = "Connector::connect() could not connect to ";
		msg += ("object '" +object->getName()+ "' of type '"
			+ object->getConcreteClassName() + "' because Connector expected "
			+ _type + " type.");
		throw( Exception(msg, __FILE__, __LINE__) );
	}
}

void Connector::connectAtStage( const SimTK::Stage& connectAtStage )
{

}

void Connector::setConnectorType( const std::string& type )
{

}


const Object& Connector::getConnectedTo() const
{
	return *_source;
}


} // end of namespace OpenSim