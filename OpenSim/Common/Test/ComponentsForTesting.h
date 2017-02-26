/* -------------------------------------------------------------------------- *
 *                     OpenSim: ComponentsForTesting.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Ayman Habib, Christopher Dembia                      *
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

/** This file contains Components that may be used by multiple tests in this
 * directory, and that exist only for use in tests. See TheWorld in
 * testComponentInterface.cpp for a related class. These two classes are
 * separate in case it becomes necessary for testComponentInterface's TheWorld
 * to get additional members that would not be necessary for other test cases
 * in the library. */

#include <OpenSim/Common/Component.h>
#include <simbody/internal/SimbodyMatterSubsystem.h>
#include <simbody/internal/GeneralForceSubsystem.h>
#include <simbody/internal/Force.h>

class RootComponent : public OpenSim::Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(RootComponent, OpenSim::Component);
public:
    RootComponent() : Component() { }

    RootComponent(const std::string& fileName, bool updFromXMLNode = false)
        : Component(fileName, updFromXMLNode) {
        // Propagate XML file values to properties 
        updateFromXMLDocument();
        // add components listed as properties as sub components.
        finalizeFromProperties();
    }

    void add(OpenSim::Component* comp) {
        addComponent(comp);
    }

    // Top level connection method for this all encompassing component.
    void connect() {
        Super::finalizeConnections(*this);
    }
    void buildUpSystem(SimTK::MultibodySystem& system) { 
        connect();
        addToSystem(system);
    }

    const SimTK::SimbodyMatterSubsystem& getMatterSubsystem() const
    { return *matter; }
    SimTK::SimbodyMatterSubsystem& updMatterSubsystem() const
    { return *matter; }

    const SimTK::GeneralForceSubsystem& getForceSubsystem() const
    { return *forces; }
    SimTK::GeneralForceSubsystem& updForceSubsystem() const
    { return *forces; }

protected:
    // Component interface implementation
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        if (system.hasMatterSubsystem()){
            matter = system.updMatterSubsystem();
        }
        else{

            SimTK::SimbodyMatterSubsystem* old_matter = matter.release();
            delete old_matter;
            matter = new SimTK::SimbodyMatterSubsystem(system);

            SimTK::GeneralForceSubsystem* old_forces = forces.release();
            delete old_forces;
            forces = new SimTK::GeneralForceSubsystem(system);

            SimTK::Force::UniformGravity gravity(*forces, *matter,
                    SimTK::Vec3(0, -9.816, 0));
            fix = gravity.getForceIndex();

            system.updMatterSubsystem().setShowDefaultGeometry(true);
        }
    }

private:
    // Keep track of pointers to the underlying computational subsystems. 
    mutable SimTK::ReferencePtr<SimTK::SimbodyMatterSubsystem> matter;
    mutable SimTK::ReferencePtr<SimTK::GeneralForceSubsystem> forces;

    // keep track of the force added by the component
    mutable SimTK::ForceIndex fix;

}; // end of RootComponent


