#ifndef __ModelComponent_h__
#define __ModelComponent_h__

// ModelComponent.h
// Authors: Peter Eastman, Ajay Seth
/*
 * Copyright (c) 2009, Stanford University. All rights reserved. 
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
#include "OpenSim/Common/Object.h"
#include "SimTKsimbody.h"

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * This is the base class for any object which can be added to a Model.  It
 * defines a set of methods which objects may optionally implement to perform
 * certain functions when the Model is loaded or initialized.
 */

class OSIMSIMULATION_API ModelComponent : public Object
{
protected:
	Model* _model;
//=============================================================================
// METHODS
//=============================================================================
public:
    template <class T> friend class ModelComponentSet;
	ModelComponent();
	ModelComponent(const std::string& aFileName, bool aUpdateFromXMLNode = true) SWIG_DECLARE_EXCEPTION;
	ModelComponent(const XMLDocument* aDocument);
	ModelComponent(DOMElement* aNode);
	ModelComponent(const ModelComponent& copy);
    virtual ~ModelComponent();
    /**
     * Get the Model this object is part of.
     */
    const Model& getModel() const;
    /**
     * Get a modifiable reference to the Model this object is part of.
     */
    Model& updModel();
protected:
    /**
     * This is called after the Model has been constructed from an XML file.  
	 * Set the Model this object is part of and then so do required initialization,
	 * such as looking up references to other objects in the Model (which might not 
	 * have existed yet when this object was instantiated).
	 * Override this method as necessary but always call the parent setup to ensure
	 * all members (belonging to parent and child) are initialized.
     */
	virtual void setup(Model& model);

    /**
     * This is called when a SimTK System is being created for the Model.  It must be implemented 
     * to add appropriate elements to the System corresponding to this object.
     *
     * @param system   the System being created
     */
    virtual void createSystem(SimTK::MultibodySystem& system) const = 0;

    /**
     * This is called after a SimTK System and State have been created for the Model.  It must
     * be implementd to set initial values of state variables.
     *
     * @param state    the State to initialize
     */
    virtual void initState(SimTK::State& state) const = 0;

    /**
     * Set all default values for this object to match those in a specified State.  It must be
     * implemented/overriden to set any default values defined by each subclass.
     *
     * @param state    the State from which to take values that should become the defaults for this object
     */
    virtual void setDefaultsFromState(const SimTK::State& state) = 0;

//=============================================================================
};	// END of class ModelComponent
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ModelComponent_h__

