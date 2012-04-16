#ifndef OPENSIM_COMPONENT_SET_
#define OPENSIM_COMPONENT_SET_

// ComponentSet.h
/*
 * Copyright (c) 2012, Stanford University. All rights reserved. 
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>

namespace OpenSim {

class Model;
//=============================================================================
//=============================================================================
/**
 * A class for holding a set of miscellaneous model components.
 *
 * @authors Michael Sherman
 */

class OSIMSIMULATION_API ComponentSet 
:   public ModelComponentSet<ModelComponent> {
OpenSim_DECLARE_CONCRETE_OBJECT(ComponentSet, 
                                ModelComponentSet<ModelComponent>);

public:
    ComponentSet() : Super() {}

	explicit ComponentSet(Model& model) : Super(model) {}

	ComponentSet(Model& model, const std::string &aFileName, 
                 bool aUpdateFromXMLNode=true)
    :   Super(model, aFileName, aUpdateFromXMLNode) {}

    // default copy constructor, copy assignment, destructor

//=============================================================================
};	// END of class ComponentSet
//=============================================================================
//=============================================================================

} // namespace OpenSim

#endif // OPENSIM_COMPONENT_SET_
