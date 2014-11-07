/* -------------------------------------------------------------------------- *
 *                OpenSim:  testComponentInterface.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Common/Component.h>

using namespace OpenSim;
using namespace std;
using namespace SimTK;

class Foo;
class Bar;


class TheWorld : public Component {
	OpenSim_DECLARE_CONCRETE_OBJECT(TheWorld, Component);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
	OpenSim_DECLARE_LIST_PROPERTY(components, Component, 
		"List of internal components");

	TheWorld() : Component() {
		// Constructing own properties, connectors, inputs or connectors? Must invoke!
		constructInfrastructure();
	}

	TheWorld(const std::string& fileName, bool updFromXMLNode = false)
		: Component(fileName, updFromXMLNode) {
		// have to construct this Component's properties so that deserialization from
		// XML has a place to go.
		constructInfrastructure();
		// Propogate XML file values to properties 
		updateFromXMLDocument();
		// add components listed as properties as sub components.
		finalizeFromProperties();
	}

	void add(Component* comp) {
		// add it the property list of components that owns and serializes them
		updProperty_components().adoptAndAppendValue(comp);
	}

	// Top level connection method for all encompassing Component
	void connect() { Super::connect(*this); }
	void buildUpSystem(MultibodySystem& system) { addToSystem(system); }

	const SimbodyMatterSubsystem& getMatterSubsystem() const { return *matter; }
	SimbodyMatterSubsystem& updMatterSubsystem() const { return *matter; }

	const GeneralForceSubsystem& getForceSubsystem() const { return *forces; }
	GeneralForceSubsystem& updForceSubsystem() const { return *forces; }

protected:
	// Component interface implementation
	void finalizeFromProperties() override {
		clearComponents();
		// Mark components listed in properties as subcomponents
		for (int i = 0; i < getProperty_components().size(); ++i){
			addComponent(&upd_components(i));
		}

		Super::finalizeFromProperties();
	}
	
	void extendAddToSystem(MultibodySystem& system) const override {
		if (system.hasMatterSubsystem()){
			matter = system.updMatterSubsystem();
		}
		else{
			SimbodyMatterSubsystem* old_matter = matter.release();
			delete old_matter;
			matter = new SimbodyMatterSubsystem(system);

			GeneralForceSubsystem* old_forces = forces.release();
			delete old_forces;
			forces = new GeneralForceSubsystem(system);

			SimTK::Force::UniformGravity gravity(*forces, *matter, Vec3(0, -9.816, 0));
			fix = gravity.getForceIndex();

			system.updMatterSubsystem().setShowDefaultGeometry(true);
		}
	}

private:
	void constructProperties() override {
		constructProperty_components();
	}

private:
	// Keep track of pointers to the underlying computational subsystems. 
	mutable ReferencePtr<SimbodyMatterSubsystem> matter;
	mutable ReferencePtr<GeneralForceSubsystem> forces;

	// keep track of the force added by the component
	mutable ForceIndex fix;

}; // end of TheWorld


class Foo : public Component {
	OpenSim_DECLARE_CONCRETE_OBJECT(Foo, Component);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
	OpenSim_DECLARE_PROPERTY(mass, double, "mass (kg)");
	OpenSim_DECLARE_LIST_PROPERTY_SIZE(inertia, double, 6,
		"inertia {Ixx, Iyy, Izz, Ixy, Ixz, Iyz}");

	Foo() : Component() {
		constructInfrastructure();
        m_ctr = 0;
        m_mutableCtr = 0;
	}

	double getSomething(const SimTK::State& state) const {
        const_cast<Foo *>(this)->m_ctr++;
        m_mutableCtr++;

		return state.getTime();
	}

	SimTK::Vec3 calcSomething(const SimTK::State& state) const {
        const_cast<Foo *>(this)->m_ctr++;
        m_mutableCtr++;

		double t = state.getTime();
		return SimTK::Vec3(t, t*t, sqrt(t));
	}

	SimTK::SpatialVec calcSpatialAcc(const SimTK::State& state) const {
        const_cast<Foo *>(this)->m_ctr++;
        m_mutableCtr++;

		return getSystem().getMatterSubsystem().getMobilizedBody(bindex)
			.getBodyAcceleration(state);
	}

protected:
	/** Component Interface */
    void connect(Component& root, const bool topLevel) override {
        Super::connect(root, topLevel);
		// do any internal wiring
		world = dynamic_cast<TheWorld*>(&root);
	}

	void extendAddToSystem(MultibodySystem &system) const override {
		Super::extendAddToSystem(system);

		SimbodyMatterSubsystem& matter = system.updMatterSubsystem();

		Vec3 mInB(0.0, 1.0, 0);
		Vec3 mInP(0, 0, 0);

		SimTK::Body::Rigid bone(
			MassProperties(1, Vec3(0), Inertia::brick(0.5, 1, 0.5)));

		// Thigh connected by hip
		MobilizedBody::Pin b1ToGround(matter.updGround(), SimTK::Transform(mInP),
			bone, SimTK::Transform(mInB));

		//Pin knee connects shank
		MobilizedBody::Pin b1ToB2(b1ToGround, SimTK::Transform(mInP),
			bone, SimTK::Transform(mInB));

		bindex = b1ToB2.getMobilizedBodyIndex();
	}

private:
    int m_ctr;
    mutable int m_mutableCtr;

	void constructProperties() override {
		constructProperty_mass(1.0);
		Array<double> inertia(0.001, 6);
		inertia[0] = inertia[1] = inertia[2] = 0.1;
		constructProperty_inertia(inertia);
	}

	void constructInputs() override {
		constructInput<double>("input1", SimTK::Stage::Model);
		constructInput<Vector>("AnglesIn", SimTK::Stage::Model);

        constructInput<double>("fiberLength", SimTK::Stage::Model);
        constructInput<double>("activation", SimTK::Stage::Model);
	}

	void constructOutputs() override {
		constructOutput<double>("Output1", &Foo::getSomething,
                SimTK::Stage::Time);

		constructOutput<SimTK::Vec3>("Output2", &Foo::calcSomething,
                SimTK::Stage::Time);

		double a = 10;
		constructOutput<Vector>("Qs",
			std::bind([=](const SimTK::State& s)->Vector{return s.getQ(); }, std::placeholders::_1),
			SimTK::Stage::Position);

		constructOutput<SpatialVec>("BodyAcc",
			std::bind(&Foo::calcSpatialAcc, this, std::placeholders::_1),
			SimTK::Stage::Velocity);
	}

	// Keep indices and reference to the world
	mutable MobilizedBodyIndex bindex;
	ReferencePtr<TheWorld> world;

	
}; // End of class Foo

class Bar : public Component {
	OpenSim_DECLARE_CONCRETE_OBJECT(Bar, Component);
public:
	Bar() : Component() { constructInfrastructure(); }

	double getPotentialEnergy(const SimTK::State& state) const {
		const GeneralForceSubsystem& forces = world->getForceSubsystem();
		const Force& force = forces.getForce(fix);
		const Force::TwoPointLinearSpring& spring = 
			Force::TwoPointLinearSpring::downcast(force);
	
		return spring.calcPotentialEnergyContribution(state);
	}

protected:
	/** Component Interface */
    void connect(Component& root, const bool topLevel) override{
        Super::connect(root, topLevel);
		// do any internal wiring
		world = dynamic_cast<TheWorld*>(&root);
		// perform custom checking
		if (updConnector<Foo>("parentFoo").getConnectee()
				== updConnector<Foo>("childFoo").getConnectee()){
			string msg = "ERROR - Bar::connect()\n";
			msg += " parentFoo and childFoo cannot be the same component.";
			throw OpenSim::Exception(msg);
		}
	}

	// Copied here from Component for testing purposes.


	void extendAddToSystem(MultibodySystem& system) const override{

		GeneralForceSubsystem& forces = world->updForceSubsystem();
		SimbodyMatterSubsystem& matter = world->updMatterSubsystem();

		int nb = matter.getNumBodies();
		if (nb > 2) {
			const MobilizedBody& b1 = matter.getMobilizedBody(MobilizedBodyIndex(1));
			const MobilizedBody& b2 = matter.getMobilizedBody(MobilizedBodyIndex(2));

			Force::TwoPointLinearSpring 
				spring(forces, b1, Vec3(0.5,0,0), b2, Vec3(0.5,0,0), 10.0, 0.1);
			fix = spring.getForceIndex();
		}

        // We use these to test the Output's that are generated when we
        // add a StateVariable.
        addStateVariable("fiberLength", SimTK::Stage::Velocity);
        addStateVariable("activation", SimTK::Stage::Dynamics);

        // Create a hidden state variable, so we can ensure that hidden state
        // variables do not have a corresponding Output.
		bool hidden = true;
		addStateVariable("hiddenStateVar", SimTK::Stage::Dynamics, hidden);
	}

    void computeStateVariableDerivatives(const SimTK::State& state) const override {
        setStateVariableDerivative(state, "fiberLength", 2.0);
        setStateVariableDerivative(state, "activation", 3.0 * state.getTime());
		setStateVariableDerivative(state, "hiddenStateVar", 
			                              exp(-0.5 * state.getTime()));
    }

private:
	void constructConnectors() override{
		constructConnector<Foo>("parentFoo");
		constructConnector<Foo>("childFoo");
	}

	void constructOutputs() override {
		constructOutput<double>("PotentialEnergy",
		std::bind(&Bar::getPotentialEnergy, this, std::placeholders::_1),
		SimTK::Stage::Velocity);
	}

	// keep track of the force added by the component
	mutable ForceIndex fix;
	ReferencePtr<TheWorld> world;

}; // End of class Bar

// Create 2nd level derived class to verify that Component interface
// hold up.
class CompoundFoo : public Foo {
	OpenSim_DECLARE_CONCRETE_OBJECT(CompoundFoo, Foo);
public:
	//=============================================================================
	// PROPERTIES
	//=============================================================================
	OpenSim_DECLARE_PROPERTY(Foo1, Foo, "1st Foo of CompoundFoo");
	OpenSim_DECLARE_PROPERTY(Foo2, Foo, "2nd Foo of CompoundFoo");
	OpenSim_DECLARE_PROPERTY(scale1, double, "Scale factor for 1st Foo");
	OpenSim_DECLARE_PROPERTY(scale2, double, "Scale factor for 2nd Foo");

	CompoundFoo() : Foo() {
		constructInfrastructure();
	}

	// API calls can change the component properties and underlying components
	// before proceding to do any calculations make sure those changes are
	// finalized.
	void finalizeChanges() { finalizeFromProperties(); }

protected:
	// Component implementation interface
	void finalizeFromProperties() override{
		// Mark components listed in properties as subcomponents
		Foo& foo1 = upd_Foo1();
		Foo& foo2 = upd_Foo2();
		
		// clear sub component designation of any previous components
		clearComponents();
		//now add them
		addComponent(&foo1);
		addComponent(&foo2);

		// update CompoundFoo's properties based on it sub Foos
		double orig_mass = get_mass();
		upd_mass() = get_scale1()*foo1.get_mass() + get_scale2()*foo2.get_mass();

		double inertiaScale = (get_mass() / orig_mass);

		for (int i = 0; i < updProperty_inertia().size(); ++i) {
			upd_inertia(i) = inertiaScale*get_inertia(i);
		}

		// enable newly added subcompenents a chance to finalize
		Super::finalizeFromProperties();
	}

private:
	void constructProperties() override {
		constructProperty_Foo1(Foo());
		constructProperty_Foo2(Foo());
		constructProperty_scale1(1.0);
		constructProperty_scale2(2.0);
	}	
}; // End of Class CompoundFoo

SimTK_NICETYPENAME_LITERAL(Foo);
SimTK_NICETYPENAME_LITERAL(Bar);

int main() {

	//Register new types for testing deserialization
	Object::registerType(Foo());
	Object::registerType(Bar());
	// Register connector objects that are in use
	Object::registerType(Connector<Foo>());
	Object::registerType(Connector<Bar>());

    try {
		// Define the Simbody system
		MultibodySystem system;

		TheWorld theWorld;
		theWorld.setName("World");
		
		// let component add its stuff to the system
		Foo& foo = *new Foo();
		foo.setName("Foo");
		theWorld.add(&foo);
		foo.set_mass(2.0);

		Foo* footTest = foo.clone();

		Bar& bar = *new Bar();
		bar.setName("Bar");
		theWorld.add(&bar);

		//Configure the connector to look for its dependency by this name
		//Will get resolved and connected automatically at Component connect
		bar.updConnector<Foo>("parentFoo").set_connected_to_name("Foo");
		bar.updConnector<Foo>("childFoo").set_connected_to_name("Foo");
		
		// add a subcomponent
		// connect internals
		try{
			theWorld.connect();
		}
		catch (const std::exception& e) {
			// Should fail to connect because child and parent Foo's are the same
			// Component and a Bar connects two Foo's.
			cout << e.what() << endl;
		}

        ComponentList<Component> worldTreeAsList = theWorld.getComponentList();
        std::cout << "list begin: " << worldTreeAsList.begin()->getName() << std::endl;
        for (ComponentList<Component>::iterator it = worldTreeAsList.begin();
            it != worldTreeAsList.end();
            ++it) {
            std::cout << "Iterator is at: " << it->getName() << std::endl;
        }

        
        std::cout << "Using range-for loop: " << std::endl;
        for (const Component& component : worldTreeAsList) {
            std::cout << "Iterator is at: " << component.getName() << std::endl;
        }
        for (auto& component : worldTreeAsList) {
            std::cout << "Iterator is at: " << component.getName() << std::endl;
        }
        
        std::cout << "Iterate over only Foo's." << std::endl;
        for (auto& component : theWorld.getComponentList<Foo>()) {
            std::cout << "Iterator is at: " << component.getName() << std::endl;
        }
        

		Foo& foo2 = *new Foo();
		foo2.setName("Foo2");
		foo2.set_mass(3.0);

		theWorld.add(&foo2);

		bar.updConnector<Foo>("childFoo").set_connected_to_name("Foo2");
		string connectorName = bar.updConnector<Foo>("childFoo").getConcreteClassName();

		// Bar should connect now
		theWorld.connect();

        std::cout << "Iterate over only Foo's." << std::endl;
        for (auto& component : theWorld.getComponentList<Foo>()) {
            std::cout << "Iterator is at: " << component.getName() << std::endl;
        }

		theWorld.buildUpSystem(system);

		// do any other input/output connections
		foo.getInput("input1").connect(bar.getOutput("PotentialEnergy"));
	
		// check how this model serializes
		string modelFile("testComponentInterfaceModel.osim");
		theWorld.print(modelFile);

		// Simbody model state setup
		State s = system.realizeTopology();

		int nu = system.getMatterSubsystem().getNumMobilities();

		//SimTK::Visualizer viz(system);
		//viz.drawFrameNow(s);
		const Vector q = Vector(s.getNQ(), SimTK::Pi/2);
		const Vector u = Vector(s.getNU(), 1.0);
		
		for (int i = 0; i < 10; ++i){
			s.updTime() = i*0.01234;
			s.updQ() = (i+1)*q/10.0;
			system.realize(s, Stage::Velocity);

			const AbstractOutput& out1 = foo.getOutput("Output1");
			const AbstractOutput& out2 = foo.getOutput("Output2");
			const AbstractOutput& out3 = foo.getOutput("Qs");
			const AbstractOutput& out4 = foo.getOutput("BodyAcc");
			const AbstractOutput& out5 = bar.getOutput("PotentialEnergy");

			cout << "=========================[Time " << s.getTime() << "s]======================="<<endl;
			cout << out1.getName() <<"|"<< out1.getTypeName() <<"|"<< out1.getValueAsString(s) << endl;
			cout << out2.getName() <<"|"<< out2.getTypeName() <<"|"<< out2.getValueAsString(s) << endl;
			cout << out3.getName() <<"|"<< out3.getTypeName() <<"|"<< out3.getValueAsString(s) << endl;
			
			system.realize(s, Stage::Acceleration);
			cout << out4.getName() <<"|"<< out4.getTypeName() <<"|"<< out4.getValueAsString(s) << endl;
			cout << out5.getName() <<"|"<< out5.getTypeName() <<"|"<< out5.getValueAsString(s) << endl;

			//viz.report(s); 
			system.realize(s, Stage::Report);

			cout << "foo.input1 = " << foo.getInputValue<double>(s, "input1") << endl;
		}

		MultibodySystem system2;
		TheWorld *world2 = new TheWorld(modelFile);
		
		world2->updComponent("Bar").getConnector<Foo>("childFoo");
        // We haven't called connect yet, so this connection isn't made yet.
		ASSERT_THROW(OpenSim::Exception,
                world2->updComponent("Bar").getConnectee<Foo>("childFoo");
                );

		ASSERT(theWorld == *world2, __FILE__, __LINE__,
			"Model serialization->deserialization FAILED");

		world2->setName("InternalWorld");
		world2->connect();

		world2->updComponent("Bar").getConnector<Foo>("childFoo");
        ASSERT("Foo2" ==
                world2->updComponent("Bar").getConnectee<Foo>("childFoo").getName());

		world2->buildUpSystem(system2);
		s = system2.realizeTopology();
	
		world2->print("clone_" + modelFile);

		// Test copy assignment
		TheWorld world3;
		world3= *world2;
		world3.getComponent("Bar").getConnector<Foo>("parentFoo");

		ASSERT(world3 == (*world2), __FILE__, __LINE__, 
			"Model copy assignment FAILED");

		// Add second world as the internal model of the first
		theWorld.add(world2);
		theWorld.connect();

		Bar& bar2 = *new Bar();
		bar2.setName("bar2");
		CompoundFoo& compFoo = *new CompoundFoo();
		compFoo.setName("BigFoo");

		compFoo.set_Foo1(foo);
		compFoo.set_Foo2(foo2);
		compFoo.finalizeChanges();
	
		world3.add(&compFoo);
		world3.add(&bar2);

		//Configure the connector to look for its dependency by this name
		//Will get resolved and connected automatically at Component connect
		bar2.updConnector<Foo>("parentFoo").set_connected_to_name("BigFoo");
		bar2.updConnector<Foo>("childFoo").set_connected_to_name("Foo");

		world3.connect();
		world3.print("Compound_" + modelFile);

		MultibodySystem system3;
		theWorld.buildUpSystem(system3);
		//SimTK::Visualizer viz2(system2);

        // Connect our state variables.
        foo.getInput("fiberLength").connect(bar.getOutput("fiberLength"));
        foo.getInput("activation").connect(bar.getOutput("activation"));
        // Since hiddenStateVar is a hidden state variable, it has no
        // corresponding output.
        ASSERT_THROW( OpenSim::Exception,
            const AbstractOutput& out = bar.getOutput("hiddenStateVar") );

		s = system3.realizeTopology();

        bar.setStateVariable(s, "fiberLength", 1.5);
        bar.setStateVariable(s, "activation", 0);

		int nu3 = system3.getMatterSubsystem().getNumMobilities();

		// realize simbody system to velocity stage
		system3.realize(s, Stage::Velocity);

		RungeKuttaFeldbergIntegrator integ(system3);
		integ.setAccuracy(1.0e-3);

		TimeStepper ts(system3, integ);
		ts.initialize(s);
		ts.stepTo(1.0);
		s = ts.getState();

        // Check the result of the integration on our state variables.
        ASSERT_EQUAL(3.5, bar.getOutputValue<double>(s, "fiberLength"), 1e-10);
        ASSERT_EQUAL(1.5, bar.getOutputValue<double>(s, "activation"), 1e-10);

        // Ensure the connection works.
        ASSERT_EQUAL(3.5, foo.getInputValue<double>(s, "fiberLength"), 1e-10);
        ASSERT_EQUAL(1.5, foo.getInputValue<double>(s, "activation"), 1e-10);

		theWorld.print("Doubled" + modelFile);
	}
    catch (const std::exception& e) {
		cout << e.what() <<endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
