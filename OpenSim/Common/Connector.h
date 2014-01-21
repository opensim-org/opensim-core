#ifndef OPENSIM_CONNECTOR_H_
#define OPENSIM_CONNECTOR_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Connector.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                           *
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

/** @file
 * This file defines the Connector class, which formalizes the dependency of 
 * of a Component on another Object/Component in order to operate, BUT it does 
 * not own it. While Components can be composites (of multiple components) 
 * they often depend on unrelated objects/components that are define elsewhere.
 * For example a Joint connects two bodies together, neither or at best the
 * child body can own the joint. It must have a "Connector" to a parent body
 * that already exists. The maintenance of the dependency and the run-time 
 * verification of the existance of the parent is the task of the Connector.
 */

// INCLUDES
#include "OpenSim/Common/Object.h"

namespace OpenSim {

//=============================================================================
//                            OPENSIM CONNECTOR
//=============================================================================
/**
 * A Connector formalizes the need for a connection between its owner and a
 * dependent object, without owning the object it is connected to. The purpose
 * of a Connector is to specify: 1) the object/type the current object is 
 * dependent on, 2) by when (what stage) the connector must be connected in
 * order for the component to function, 3) whether it is connected or not.
 *
 * For example, a Joint has two Connectors one for each of the Bodies that a
 * Joint joins. The type for these connectors is Body and any attempt to 
 * connect to a non-Body object will throw an exception.
 * The connectAt Stage is Topology. That is the Joint's connection to a Body 
 * must be performed at the Topology system stage, and any attempt to change 
 * the connection status (either connect of disconnect) will result in an
 * Exception being thrown.
 *
 * @author  Ajay Seth
 */

class OSIMCOMMON_API Connector : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(Connector, Object);

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	/** Default constructor */
	Connector();
	/** Convenience constructor 
	    Create a Connector with specified type and stage at which it
		can be connected.
	@param type				Object type (string) for this Connector. 
	@param connectAtStage	Stage at which Connector can be connected. */
	Connector(const std::string& type, const SimTK::Stage& connectAtStage);

	virtual ~Connector() {};

	//--------------------------------------------------------------------------
	// CONFIGURE
	//--------------------------------------------------------------------------
	/** set the type of Object this connector connects to*/
	void setConnectorType(const std::string& type);
	/** set the stage by when the connection should be made */
	void connectAtStage(const SimTK::Stage& connectAtStage);

	//--------------------------------------------------------------------------
	// CONNECTION
	//--------------------------------------------------------------------------
	/** Establish a connection to an existing object of the required type.*/
	void connect(const Object* object);
	/** Return the object this Connector is connected to. */
	const Object& getConnectedTo() const;
	/** Release the Connector from the current object  which is connected */
	void disconnect();

	//--------------------------------------------------------------------------
	// STATUS
	//--------------------------------------------------------------------------
    /** Is the Connector connected to an object? */
	bool isConnected() const;


private:
	SimTK::ReferencePtr<const Object> _source;
	std::string _type;
	SimTK::Stage _connectAtStage;
	bool _isConnected;

//=============================================================================
};	// END class Connector

//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif  // OPENSIM_CONNECTOR_H_
