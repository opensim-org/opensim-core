// testSimulation.cpp
// Author:  Frank C. Anderson


// INCLUDES
#include <iostream>
#include <string>
#include <iostream>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/Math.h>
#include <OpenSim/Tools/Exception.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/Array.h>
#include <OpenSim/Tools/PropertyInt.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyIntArray.h>
#include <OpenSim/Tools/PropertyStrArray.h>
#include <OpenSim/Tools/PropertyObjArray.h>
#include <OpenSim/Simulation/Control/ControlLinearNode.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlConstant.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/Model/GeneralizedForce.h>
#include <OpenSim/Actuators/GeneralizedForceAtv.h>
#include <OpenSim/Simulation/Model/ActuatorSet.h>
#include <OpenSim/Actuators/LinearSetPoint.h>
#include <OpenSim/Actuators/PolynomialSetPoint.h>
#include <OpenSim/Simulation/Model/ContactForceSet.h>




using namespace OpenSim;
using namespace std;


// DECLARATIONS
void TestControlLinearNode();
void TestControlLinear();
void TestControlSet1();
void TestControlSet2();
void TestControlSerialization();
void TestControlSetSerialization();
void TestActuatorSetSerialization();
void TestContactForceSetSerialization();


//_____________________________________________________________________________
/**
 * Test the rdSimulation library.
 */
int main(int argc, char* argv[])
{
	// PARSE COMMAND LINE

	// CONTROLS
	//TestControlLinearNode();
	//TestControlLinear();
	//TestControlSet1();
	//TestControlSet2();
	//TestControlSerialization();
	//TestControlSetSerialization();
	//TestActuatorSetSerialization();
	//TestContactForceSetSerialization();

	return(0);
}


//_____________________________________________________________________________
/**
 * Test the serialization of a contact force set.
 */
void TestContactForceSetSerialization()
{

	//=============
	// OUTPUT
	//=============
	//-------------
	// EMPTY CONTROL SET
	//-------------
	ContactForceSet contactSet1;
	contactSet1.setName("contactSet1");
	
	//-------------
	// LinearSetPoint
	//-------------
	LinearSetPoint linear;
	linear.setName("s1");
	contactSet1.append(&linear);

	//-------------
	// PolynomialSetPoint
	//-------------
	PolynomialSetPoint poly;
	poly.setName("s2");
	contactSet1.append(&poly);

	//-------------
	// PRINT
	//-------------
	contactSet1.print("contactSet1.ctx");


	//=============
	// INPUT
	//=============
	//-------------
	// REGISTER OBJECT TYPES
	//-------------
	Object::RegisterType(linear);
	Object::RegisterType(poly);

	//-------------
	// READ CONTACT SET
	//-------------
	ContactForceSet contactSet2("fallingBlock.ctx");
	contactSet2.setName("contactSet2");
	int index=0;
	ContactForce *ctx = (ContactForce*)contactSet2.get(index);
	cout<<"\n\n"<<ctx->getName()<<endl;
	ctx->check();
	PropertySet &propSet = ctx->getPropertySet();
	Property *propA = propSet.get("body_A");
	Property *propB = propSet.get("body_B");
	cout<<endl;
	cout<<"\nproperty address "<<propA<<endl;
	cout<<propA<<": "<<propA->getValueInt()<<endl;
	cout<<"\nproperty address "<<propB<<endl;
	cout<<propB<<": "<<propB->getValueInt()<<endl;
	cout<<endl;

	propA->setValue((int)3);
	cout<<"\nproperty address "<<propA<<endl;
	cout<<propA<<": "<<propA->getValueInt()<<endl;

	ctx->check();


	//-------------
	// PRINT
	//-------------
	contactSet2.updateXMLNode(NULL);
	contactSet2.print("contactSet2.act");
}

//_____________________________________________________________________________
/**
 * Test the serialization of an actuator set.
 */
void TestActuatorSetSerialization()
{

	//=============
	// REGISTER
	//=============
	Object::RegisterType(Force());
	Object::RegisterType(GeneralizedForce());
	Object::RegisterType(GeneralizedForceAtv());

	
	//=============
	// OUTPUT
	//=============
	//-------------
	// EMPTY CONTROL SET
	//-------------
	ActuatorSet actuatorSet1;
	actuatorSet1.setName("actuatorSet1");
	actuatorSet1.setMemoryOwner(false);
	
	//-------------
	// 0 Force
	//-------------
	Force force1;
	force1.setName("force1");
	actuatorSet1.append(&force1);

	//-------------
	// 1 Force
	//-------------
	Force force2;
	force2.setName("force2");
	actuatorSet1.append(&force2);

	//-------------
	// 2 GeneralizedForceAtv
	//-------------
	GeneralizedForceAtv genForceAtv1;
	genForceAtv1.setName("genForceAtv1");
	actuatorSet1.append(&genForceAtv1);

	//-------------
	// 3 GeneralizedForce
	//-------------
	GeneralizedForce genForce1;
	genForce1.setName("genForce1");
	actuatorSet1.append(&genForce1);

	//-------------
	// 4 GeneralizedForceAtv
	//-------------
	GeneralizedForceAtv genForceAtv2;
	genForceAtv2.setName("genForceAtv2");
	actuatorSet1.append(&genForceAtv2);

	//-------------
	// 5 Force
	//-------------
	Force force3;
	force3.setName("force3");
	actuatorSet1.append(&force3);

	//-------------
	// PRINT
	//-------------
	actuatorSet1.print("actuatorSet1.act");


	//=============
	// INPUT
	//=============

	//-------------
	// READ
	//-------------
	ActuatorSet actuatorSet2("actuatorSet1.act");
	actuatorSet2.setName("actuatorSet2");
	actuatorSet2.updateXMLNode(NULL);

	//-------------
	// PRINT
	//-------------
	actuatorSet2.print("actuatorSet2.act");


	//=============
	// ASSIGNMENT
	//=============
	ActuatorSet actuatorSet;
	actuatorSet = actuatorSet2;
	actuatorSet.setMemoryOwner(true);
	actuatorSet.setName("AssignedToActuatorSet2");
	actuatorSet.print("actuatorSetAssigned.act");
}


//_____________________________________________________________________________
/**
 * Test the serialization of a control set.
 */
void TestControlSetSerialization()
{

	//=============
	// OUTPUT
	//=============
	//-------------
	// EMPTY CONTROL SET
	//-------------
	ControlSet controlSet1;
	controlSet1.setName("testSet1");
	
	//-------------
	// CONSTANT
	//-------------
	// 0
	ControlConstant cntrConst;
	cntrConst.setName("tf");
	controlSet1.append(&cntrConst);

	//-------------
	// LINEAR
	//-------------
	// 1
	ControlLinear cntrLin;
	cntrLin.setName("soleus");
	controlSet1.append(&cntrLin);
	// 2
	cntrLin.setName("gastroc");
	controlSet1.append(&cntrLin);


	//-------------
	// GET & SET CONTROL VALUES
	//-------------
	int i;
	double t;
	Array<double> x(0.0);
	controlSet1.getControlValues(0.0,x);
	cout<<x<<endl;
	for(t=1.0;t<=10.0;t++) {
		for(i=0;i<x.getSize();i++) x[i] = t;
		controlSet1.setControlValues(t,x);
	}
	controlSet1.getControlValues(1.1,x);
	cout<<x<<endl;

	//-------------
	// PRINT
	//-------------
	controlSet1.print("controlSet1.ctr");


	//=============
	// INPUT
	//=============
	//-------------
	// REGISTER OBJECT TYPES
	//-------------
	Object::RegisterType(ControlConstant());
	Object::RegisterType(ControlLinear());
	Object::RegisterType(ControlLinearNode());

	// LOAD
	ControlSet controlSet2("controlSet1.ctr");

	// GET VALUES
	controlSet2.setControlValues(0.0,x);
	controlSet2.getControlValues(0.0,x);
	cout<<x<<endl;
	controlSet2.getControlValues(8.4,x);
	cout<<x<<endl;
	controlSet2.setControlValues(4.2,x);
	controlSet2.getControlValues(4.2,x);
	cout<<x<<endl;

	// PRINT
	controlSet2.setName("testSet2");
	controlSet2.updateXMLNode(NULL);
	controlSet2.print("controlSet2.ctr");


	//=============
	// INPUT WITH DEFAULTS
	//=============
	//-------------
	// READ CONTROL SET
	//-------------
	ControlSet controlSetDefaults("controlSetWithDefaults.ctr");
	controlSetDefaults.setName("testSetDefaults");
	controlSetDefaults.updateXMLNode(NULL);

	//-------------
	// PRINT
	//-------------
	controlSetDefaults.print("controlSetDefaults.ctr");
}


//_____________________________________________________________________________
/**
 * Test the serialization of the control types.
 * Right now only ControlConstant and ControlLinear are tested.
 */
void TestControlSerialization()
{
	//-------------
	// CONSTANT
	//-------------
	ControlConstant cntrConst;
	cntrConst.print("const.ctr");

	// MEMORY LEAK TESTING
	ArrayPtrs<Object> nodes;
	nodes.append( new ControlLinearNode() );
	PropertyObjArray *objArray = new PropertyObjArray("nodes",nodes);
	delete objArray;

	//-------------
	// LINEAR
	//-------------
	ControlLinear cntrLin;
	PropertySet &properties = cntrLin.getPropertySet();
	Property *property = properties.get("nodes");
	ArrayPtrs<Object> &value = property->getValueObjArray();
	cout << "value size = " << value.getSize() << endl;
	cntrLin.print("linear.ctr");
}

//_____________________________________________________________________________
/**
 * Test the parameter methods of ControlSet.
 */
void TestControlSet2()
{
	IO::SetPrecision(8);

	// REGISTER CONTROL TYPES
	ControlConstant controlConstant;
	ControlLinear controlLinear;
	ControlSet::RegisterType(controlConstant);
	ControlSet::RegisterType(controlLinear);

	// READ IN TEST FILE
	ControlSet controlSet("test.ctr");

	// GET PARAMETER LISTS
	int i,j;
	// Lower, Upper
	double tLower=Math::MINUS_INFINITY,tUpper=10.0;
	Array<int> list(-1);
	controlSet.getParameterList(tLower,tUpper,list);
	printf("parameters (%lf,%lf):",tLower,tUpper);
	for(i=0;i<list.getSize();i++) {
		printf(" %d",list[i]);
	}
	printf("\n\n");
	// Time
	double t = 0.41;
	controlSet.getParameterList(t,list);
	printf("parameters (%lf):",t);
	for(i=0;i<list.getSize();i++) {
		printf(" %d",list[i]);
	}
	printf("\n\n");

	// ADD A BUNCH OF CONTROL PARAMETERS
	double dt = 0.1;
	Array<double> x(0.0);
	x.setSize(controlSet.getSize());
	for(t=0.6,i=0;i<200;i++,t+=dt) {
		for(j=0;j<x.getSize();j++) {
			x[j] = (double)i;
		}
		controlSet.setControlValues(t,x);
	}
	controlSet.print("testLarge.ctr");
	ControlSet controlSet2("testLarge.ctr");
	controlSet2.print("testLargeCopy.ctr");

	// GET PARAMETERS AND SET PARAMETERS- A PERFORMANCE TEST
	// List
	tLower=20.4,tUpper=20.5;
	controlSet.getParameterList(tLower,tUpper,list);
	printf("parameters (%lf,%lf):",tLower,tUpper);
	for(i=0;i<list.getSize();i++) {
		printf(" %d",list[i]);
	}
	printf("\n\n");
	// Get
	Array<double> p(0.0);
	controlSet.getParameterValues(p,&list);
	for(i=0;i<p.getSize();i++) {
		printf("p[%d] = %lf\n",list[i],p[i]);
	}
	// Set
	for(i=0;i<p.getSize();i++) {
		p[i] = 10000.0;
	}
	controlSet.setParameterValues(p,&list);
	controlSet.print("testLargeAltered.ctr");
}

//_____________________________________________________________________________
/**
 * Test basic features of ControlSet.
 */
void TestControlSet1()
{
	IO::SetScientific(true);

	// REGISTER CONTROL TYPES
	ControlConstant controlConstant;
	ControlLinear controlLinear;
	ControlSet::RegisterType(controlConstant);
	ControlSet::RegisterType(controlLinear);

	// PRINT DEFAULT FILE
	ControlSet defaultSet;
	defaultSet.setName("test");
	defaultSet.print("default.ctr");

	// READ IN AND WRITE OUT TEST FILE
	ControlSet controlSet("test.ctr");
	controlSet.print("testCopy.ctr");

	// GET A PARTICULAR CONTROL
	int index = controlSet.getIndex("targetDT");
	Control *control = controlSet.get(index);
	if(control!=NULL) {
		printf("targetDT = %lf\n",control->getControlValue(0.0));
	} else {
		printf("Control for ankleVelocity not found.\n");
	}

	// GET ACTUATOR CONTROLS
	double t = 0.15;
	Array<double> x(0.0);
	Array<int> list(-1);
	controlSet.getControlList("ControlLinear",list);
	controlSet.getControlValues(t,x,false);
	int i;
	printf("t=%lf\n",t);
	for(i=0;i<list.getSize();i++) {
		control = controlSet.get(list[i]);
		cout<<control->getName()<<": "<<x[list[i]]<<endl;
	}

	// GET PARAMETERS
	Array<double> p(0.0);
	controlSet.getParameterValues(p,false);
	printf("parameters: ");
	for(i=0;i<p.getSize();i++) {
		printf(" %lf",p[i]);
	}
	printf("\n");

	// ALTER PARAMETERS
	p[0] = -1.21;
	p[1] = 100.0;
	p[2] = 200.0;
	p[3] = 400.0;
	p[4] = 500.0;
	p[5] = 600.0;
	p[6] = 700.0;
	p[7] = 800.0;
	p[8] = 900.0;
	controlSet.setParameterValues(p,false);

	// GET CONTROLS AGAIN
	index = controlSet.getIndex("targetDT");
	printf("targetDT = %lf\n",controlSet.get(index)->getControlValue(t));
	controlSet.getControlList("ControlLinear",list);
	controlSet.getControlValues(t,x);
	printf("t=%lf\n",t);
	for(i=0;i<list.getSize();i++) {
		control = controlSet.get(list[i]);
		cout<<control->getName()<<": "<<x[list[i]]<<endl;
	}

}

///_____________________________________________________________________________
/**
 * Test ControlLinear.
 */
void TestControlLinear()
{
	IO::SetScientific(true);

	// CONSTRUCT
	ControlLinear control;

	// SET VALUES
	int i,n=10;
	for(i=0;i<n;i++) {
		control.setControlValue((double)i,(double)i);
	}

	// GET VALUES
	double t,dt=0.1;
	printf("\nsize = %ld\n",control.getNumParameters());
	for(t=-1.0;t<=n;t+=dt) {
		printf("%lf %lf\n",t,control.getControlValue(t));
	}

	// CHANGE CONTROL VALUES
	for(i=0;i<n;i++) {
		control.setControlValue((double)i,2.0*(double)i);
	}
	control.setControlValue(0.5,-10.0);

	// GET VALUES
	printf("\nsize = %ld\n",control.getNumParameters());
	for(t=-1.0;t<=n;t+=dt) {
		printf("%lf %lf\n",t,control.getControlValue(t));
	}


	printf("\n\n");
}

//_____________________________________________________________________________
/**
 * Test ControlLinearNode.
 */
void TestControlLinearNode()
{
	IO::SetScientific(true);

	// ARRAY OF NODES
	ControlLinearNode node;
	Array<ControlLinearNode> nodes(node);

	// APPEND TO ARRAY
	int i,n=10;
	for(i=0;i<n;i++) {
		node.setTime((double)i);
		nodes.append(node);
	}

	// PRINT
	char *tmp;
	for(i=0;i<nodes.getSize();i++) {
		tmp = nodes[i].toString();
		printf("node[%d]: %s\n",i,tmp);
		if(tmp!=NULL) delete[] tmp;
	}

	// FIND NODE
	node.setTime(2.9);
	int index = nodes.searchBinary(node);
	printf("node[%d] = %lf\n",index,nodes[index].getTime());

	// EQUALITY
	ControlLinearNode n1(0.0),n2(1.0e-14);
	printf("\nEQUALITY TEST\n");
	printf("n1: %s\n",n1.toString());
	printf("n2: %s\n",n2.toString());
	printf("\nnode equality tolerance = %le\n",node.GetEqualityTolerance());
	if(n1==n2) printf("equal = true\n");
	else printf("equal = false\n");

	node.SetEqualityTolerance(0.0);
	printf("\nnode equality tolerance = %le\n",node.GetEqualityTolerance());
	if(n1==n2) printf("equal = true\n");
	else printf("equal = false\n");

	node.SetEqualityTolerance(0.1);
	printf("\nnode equality tolerance = %le\n",node.GetEqualityTolerance());
	if(n1==n2) printf("equal = true\n");
	else printf("equal = false\n");


	printf("\n\n");
}
