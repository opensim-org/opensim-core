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
		constructInfrastructure();
	}

	TheWorld(const std::string& fileName, bool updFromXMLNode = false)
		: Component(fileName, updFromXMLNode) {
		constructInfrastructure();
		updateFromXMLDocument();
	}

	explicit TheWorld(SimTK::Xml::Element& element) : Component(element) {
		constructInfrastructure();
	}

	void add(Component* comp) { 
		addComponent(comp);
		updProperty_components().adoptAndAppendValue(comp);
	}
	void connect() { connect(*this); }
	void buildUpSystem(MultibodySystem& system) { addToSystem(system); }

	const SimbodyMatterSubsystem& getMatterSubsystem() const { return *matter; }
	SimbodyMatterSubsystem& updMatterSubsystem() const { return *matter; }

	const GeneralForceSubsystem& getForceSubsystem() const { return *forces; }
	GeneralForceSubsystem& updForceSubsystem() const { return *forces; }

protected:
	void connect(Component& root) OVERRIDE_11 {
		Super::connect(root);
	}

	void addToSystem(MultibodySystem& system) const OVERRIDE_11 {
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

		Super::addToSystem(system);
	}

private:
	void constructProperties() OVERRIDE_11 {
		constructProperty_components();
	}
	void constructStructuralConnectors() OVERRIDE_11 {
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

	Foo() : Component() {  constructInfrastructure(); }

	Foo(const std::string& fileName, bool updFromXMLNode = true)
		: Component(fileName, updFromXMLNode) {
		constructInfrastructure();
	}

	explicit Foo(SimTK::Xml::Element& element) : Component(element) 
	{ constructInfrastructure(); }

	double getSomething(const SimTK::State& state){
		return state.getTime();
	}

	SimTK::Vec3 calcSomething(const SimTK::State& state){
		double t = state.getTime();
		return SimTK::Vec3(t, t*t, sqrt(t));
	}

	SimTK::SpatialVec calcSpatialAcc(const SimTK::State& state){
		return getSystem().getMatterSubsystem().getMobilizedBody(bindex)
			.getBodyAcceleration(state);
	}

protected:
	/** Component Interface */
	void connect(Component& root) OVERRIDE_11 {
		Super::connect(root);
		// do any internal wiring
		world = dynamic_cast<TheWorld*>(&root);
	}

	void addToSystem(MultibodySystem &system) const OVERRIDE_11 {
		Super::addToSystem(system);

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
	void constructProperties() OVERRIDE_11{
		constructProperty_mass(1.0);
		Array<double> inertia(0.001, 6);
		inertia[0] = inertia[1] = inertia[2] = 0.1;
		constructProperty_inertia(inertia);
	}

	void constructInputs() OVERRIDE_11 {
		constructInput<double>("input1", SimTK::Stage::Model);
		constructInput<Vector>("AnglesIn", SimTK::Stage::Model);
	}

	void constructOutputs() OVERRIDE_11 {
		constructOutput<double>("Output1",
		std::bind(&Foo::getSomething, this, std::placeholders::_1),
			SimTK::Stage::Time);

		constructOutput<SimTK::Vec3>("Output2",
			std::bind(&Foo::calcSomething, this, std::placeholders::_1),
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

	Bar(const std::string& fileName, bool updFromXMLNode = true)
		: Component(fileName, updFromXMLNode) {
		constructInfrastructure();
	}

	explicit Bar(SimTK::Xml::Element& element) : Component(element) 
	{ constructInfrastructure(); }

	double getPotentialEnergy(const SimTK::State& state) const {
		const GeneralForceSubsystem& forces = world->getForceSubsystem();
		const Force& force = forces.getForce(fix);
		const Force::TwoPointLinearSpring& spring = 
			Force::TwoPointLinearSpring::downcast(force);
	
		return spring.calcPotentialEnergyContribution(state);
	}

protected:
	/** Component Interface */
	void connect(Component& root) OVERRIDE_11{
		Super::connect(root);
		// do any internal wiring
		world = dynamic_cast<TheWorld*>(&root);
		// perform custom checking
		if (updConnector<Foo>("parentFoo").getConectee()
				== updConnector<Foo>("childFoo").getConectee()){
			string msg = "ERROR - Bar::connect()\n";
			msg += " parentFoo and childFoo cannot be the same component.";
			throw OpenSim::Exception(msg);
		}
	}
	void addToSystem(MultibodySystem& system) const OVERRIDE_11{
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
	}

private:
	void constructStructuralConnectors() OVERRIDE_11{
		constructStructuralConnector<Foo>("parentFoo");
		constructStructuralConnector<Foo>("childFoo");
	}

	void constructOutputs() OVERRIDE_11 {
		constructOutput<double>("PotentialEnergy",
		std::bind(&Bar::getPotentialEnergy, this, std::placeholders::_1),
		SimTK::Stage::Velocity);
	}

	// keep track of the force added by the component
	mutable ForceIndex fix;
	ReferencePtr<TheWorld> world;

}; // End of class Bar

SimTK_NICETYPENAME_LITERAL(Foo);
SimTK_NICETYPENAME_LITERAL(Bar);

int main() {

	//Register new types for testing deserialization
	Object::registerType(Foo());
	Object::registerType(Bar());
	// Register connector objects that are in use
	Object::registerType(Connector<Foo>());
	Object::registerType(Connector<Bar>());

	Object::setDebugLevel(5);

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

		Bar& bar = *new Bar();
		bar.setName("Bar");
		theWorld.add(&bar);

		//Configure the connector to look for its dependency by this name
		//Will get resolved and connected automatically at Component connect
		bar.updConnector<Foo>("parentFoo").connect(foo);
		bar.updConnector<Foo>("childFoo").connect(foo);
		
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

		Foo& foo2 = *new Foo();
		foo2.setName("Foo2");
		foo2.set_mass(3.0);

		theWorld.add(&foo2);

		//bar.updConnector<Foo>("childFoo").set_connected_to_name("Foo2");
		bar.updConnector<Foo>("childFoo").connect(foo2);
		string connectorName = bar.updConnector<Foo>("childFoo").getConcreteClassName();

		// Bar should connect now
		theWorld.connect();
		theWorld.buildUpSystem(system);

		// do any other input/output connections
		foo.getInput("input1").connect(bar.getOutput("PotentialEnergy"));
	
		// check how this model serializes
		string modelFile("testComponentInterfaceModel.osim");
		theWorld.print(modelFile);

		// Simbody model state setup
		State s = system.realizeTopology();

		//SimTK::Visualizer viz(system);
		//viz.drawFrameNow(s);
		Vector q = Vector(s.getNQ(), SimTK::Pi/2);
		Vector u = Vector(s.getNU(), 1.0);
		
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

		TheWorld *world2 = new TheWorld(modelFile);
		ASSERT(theWorld == *world2, "Model serialization->deserialization FAILED");
		world2->print("clone_" + modelFile);

		// Add second world as the internal model of the first
		world2->setName("InternalWorld");
		theWorld.add(world2);
		theWorld.connect();

		MultibodySystem system2;
		theWorld.buildUpSystem(system2);
		//SimTK::Visualizer viz2(system2);
		
		s = system2.realizeTopology();

		// realize simbody system to velocity stage
		system2.realize(s, Stage::Velocity);

		RungeKuttaFeldbergIntegrator integ(system2);
		integ.setAccuracy(1.0e-3);

		TimeStepper ts(system2, integ);
		ts.initialize(s);
		ts.stepTo(1.0);
		s = ts.getState();

		theWorld.print("Doubled" + modelFile);
	}
    catch (const std::exception& e) {
		cout << e.what() <<endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
