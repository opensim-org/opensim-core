// testTools.cpp
// Author:  Frank C. Anderson
/*
* Copyright (c)  2005, Stanford University, All rights reserved. 
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
#include <iostream>
#include <string>
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Tools/Math.h>
#include <OpenSim/Common/Signal.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Line.h>
#include <OpenSim/Common/Plane.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/SIMMUtilities.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyIntArray.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/PropertySet.h>
#include <OpenSim/Common/RootSolver.h>
#include "rdSerializableObject.h"
#include "rdSerializableObject2.h"
#include "ExampleVectorFunctionUncoupledNxN.h"




using namespace OpenSim;
using namespace std;


// DECLARATIONS
void TestStorage();
void TestGCVSplineSet();
void TestXML();
void TestGeometry();
void TestSIMMUtilities();
void TestExceptions();
void ThrowException();
void TestVector();
void TestArray();
void TestProperty();
void TestPropertySet();
void TestSerialization();
void TestSignal();
int TestRootSolver();
//_____________________________________________________________________________
/**
 * Test the osimCommon library.
 */
int main(int argc, char* argv[])
{
	// PARSE COMMAND LINE

	// STORAGE
	//TestStorage();

	// XML
	//TestXML();

	// GEOMETRY
	//TestGeometry();

	// TEST GCVSplineSet
	//TestGCVSplineSet();

	// TEST SIMM UTILITIES
	//TestSIMMUtilities();

	// EXCEPTIONS
	//TestExceptions();

	// VECTOR
	//TestVector();

	// ARRAY
	//TestArray();

	// PROPERTY
	//TestProperty();

	// PROPERTY SET
	//TestPropertySet();

	// SERIALIZATION
	//TestSerialization();

	// SIGNAL
	//TestSignal();

	// ROOT SOLVER

	return(TestRootSolver());
}


//_____________________________________________________________________________
/**
 * Test the RootSolver class.
 */
int TestRootSolver()
{
	// CONSTRUCT THE UNCOUPLED VECTOR FUNCTION
	int N = 101;
	ExampleVectorFunctionUncoupledNxN function(N);

	// EVALUATE THE FUNCTION
	cout<<"\n\nEvaluate the function:\n";
	Array<double> x(0.0,N),y(0.0,N);
	function.evaluate(&x[0],&y[0]);
	cout<<"x:\n";
	cout<<x<<endl;
	cout<<"y:\n";
	cout<<y<<endl;

	// ROOT SOLVE
	Array<double> a(-1.0,N),b(1.0,N),tol(1.0e-6,N);
	Array<double> roots(0.0,N);
	RootSolver solver(&function);
	roots = solver.solve(a,b,tol);
	cout<<endl<<endl<<"-------------"<<endl;
	cout<<"roots:\n";
	cout<<roots<<endl<<endl;
	bool success = true;
	for (int i=0; i <= 100 && success; i++){
		success = (fabs(i*0.01 - roots[i])<1e-6);
	}
	return(success?0:1);
}

//_____________________________________________________________________________
/**
 * Test the methods inside class Signal.
 */
void TestSignal()
{
	// MAKE PARABOLA (x = t*t)
	int i;
	double dt = 0.01,t,x;
	Array<double> time(0.0),signal(0.0);
	for(i=0;i<1000;i++) {
		t = i*dt;
		x = t*t;
		time.append(t);
		signal.append(x);
	}
		
	// APPEND SINE WAVE (x = sin(t))
	for(i=1000;i<2000;i++) {
		t = i*dt;
		x = 10.0 * sin(t);
		time.append(t);
		signal.append(x);
	}
		
	// REDUCE POINTS
	double distance = 0.01;
	int nRemoved = Signal::ReduceNumberOfPoints(distance,time,signal);
	cout<<"\n\nRemoved "<<nRemoved<<" points.\n\n";
	distance = 0.01;
	nRemoved = Signal::ReduceNumberOfPoints(distance,time,signal);
	cout<<"\n\nRemoved "<<nRemoved<<" points.\n\n";

	// WRITE FILE
	FILE *ptr = fopen("testSignal.xls","w");
	int size = time.getSize();
	for(i=0;i<size;i++) {
		fprintf(ptr,"%lf\t%lf\n",time[i],signal[i]);
	}
	fclose(ptr);
}
//_____________________________________________________________________________
/**
 * Test the serialization of an object.
 */
void TestSerialization()
{
	// TYPE REGISTRATION
	Object::RegisterType(rdSerializableObject());
	Object::RegisterType(rdSerializableObject2());

	// OBJECT 1
	rdSerializableObject obj1;
	obj1.setName("TestObject");
	obj1.print("obj1.xml");

	// OBJECT 2
	rdSerializableObject obj2("obj1.xml");
	PropertySet &propSet1 = obj2.getPropertySet();
	rdSerializableObject2 *memberObj = (rdSerializableObject2*)
		&propSet1.get("Test_Obj")->getValueObj();
	PropertySet &propSet2 = memberObj->getPropertySet();
	Property *Bool = propSet2.get("Test_Bool2");
	Bool->setValue(true);
	obj2.updateXMLNode(NULL);
	obj2.print("obj2.xml");
}

//_____________________________________________________________________________
/**
 * Test class PropertySet.
 */
void TestPropertySet()
{
	PropertySet properties;

	cout << properties;

	// SOME PROPERRTY CLASSES
	PropertyInt i1("i1",2);
	PropertyDbl d1("d1",3.1414);
	PropertyStr s1("s1","hams");
	Array<int> intArray(0);
	intArray.setSize(10);
	intArray.set(2,1);
	PropertyIntArray ia1("ia1",intArray);
	Array<string> strArray("luck");
	strArray.setSize(8);
	strArray.set(2,"yum");
	PropertyStrArray sa1("lucky",strArray);

	// APPEND
	properties.append(i1.copy());
	properties.append(d1.copy());
	properties.append(s1.copy());
	properties.append(ia1.copy());
	properties.append(sa1.copy());
	cout << properties << endl;

	// REMOVE
	properties.remove("i1");
	cout << properties << endl;
	properties.remove("ia1");
	cout << properties << endl;

	// CLEAR
	properties.clear();
	cout << properties << endl;

	// APPEND AGAIN
	cout << "Appended properties again...\n";
	properties.append(i1.copy());
	properties.append(d1.copy());
	properties.append(s1.copy());
	properties.append(ia1.copy());
	properties.append(sa1.copy());
	cout << properties << endl;

	// GET BY INDEX
	cout << "\nTesting rdProperties.get(int):" << endl;
	int i;
	Property *property;
	for(i=0;i<properties.getSize();i++) {
		property = properties.get(i);
		cout << *property << endl;
	}

	// GET WITH OUT OF BOUNDS INDEX
	try {
		cout<<"\n\nAn exception should be generated...";
		property = properties.get(properties.getSize());
		cout << property << endl;
	} catch(Exception x) {
		x.print(cout);
	}

	// GET BY STRING
	cout << "\nTesting rdProperties.get(string):" << endl;
	try {
		Property *property;
		property = properties.get("i1");
		cout << *property << endl;
		property = properties.get("d1");
		cout << *property << endl;
		property = properties.get("s1");
		cout << *property << endl;
		property = properties.get("ia1");
		cout << *property << endl;
		property = properties.get("lucky");
		cout << *property << endl;

		cout<<"\n\nAn exception should be generated...";
		property = properties.get("bad_prop");
		cout << *property << endl;
	} catch(Exception x) {
		x.print(cout);
	}

}


//_____________________________________________________________________________
/**
 * Test property.
 */
void TestProperty()
{
	// Int
	PropertyInt pInt("nx",2);
	// Dbl
	PropertyDbl pDbl("x",2.0);
	// Str
	PropertyStr pStr("muscle","hams");
	// IntArray
	Array<int> arrayInt(2);
	arrayInt.setSize(4);
	PropertyIntArray pIntArray("BlackJack",arrayInt);
	// StrArray
	Array<string> arrayStr("suck");
	arrayStr.setSize(4);
	PropertyStrArray pStrArray("TommyJack",arrayStr);


	try {

	cout<<pInt.getTypeAsString()<<" "<<pInt.getName()<<" "<<pInt.getValueInt();
	cout<<'\n';
	cout<<pDbl.getTypeAsString()<<" "<<pDbl.getName()<<" "<<pDbl.getValueDbl();
	cout<<'\n';
	cout<<pStr.getTypeAsString()<<" "<<pStr.getName()<<" "<<pStr.getValueStr();
	cout<<'\n';
	cout<<pIntArray.getTypeAsString()<<" "<<pIntArray.getName()<<" "<<
		pIntArray.getValueIntArray();
	cout<<'\n';
	cout<<'\n';

	} catch(Exception x) {

		x.print(cout);

	}

	// Int
	cout << "\n\n<<PropertyInt>>\n";
	cout << "\nREFERENCE CAN BE USED TO CHANGE THE PROPERTY\n";
	int &IntRef = pInt.getValueInt();
	cout << "reference: " << IntRef << "\n";
	cout << "property:  " << pInt.getValueInt() << "\n";
	cout << "... change reference ...\n";
	IntRef = 0;
	cout << "reference: " << IntRef << "\n";
	cout << "property:  " << pInt.getValueInt() << "\n";

	cout << "\nCOPY CANNOT BE USED TO CHANGE THE PROPERTY\n";
	int Int;
	Int = pInt.getValueInt();
	cout << "copy:     " << Int << "\n";
	cout << "property: " << pInt.getValueInt() << "\n";
	cout << "... change copy ...\n";
	Int = 1;
	cout << "copy:     " << Int << "\n";
	cout << "property: " << pInt.getValueInt() << "\n";

	// Dbl
	cout << "\n\n<<PropertyDbl>>\n";
	cout << "\nREFERENCE CAN BE USED TO CHANGE THE PROPERTY\n";
	double &DblRef = pDbl.getValueDbl();
	cout << "reference: " << DblRef << "\n";
	cout << "property:  " << pDbl.getValueDbl() << "\n";
	cout << "... change reference ...\n";
	DblRef = 0;
	cout << "reference: " << DblRef << "\n";
	cout << "property:  " << pDbl.getValueDbl() << "\n";

	cout << "\nCOPY CANNOT BE USED TO CHANGE THE PROPERTY\n";
	double Dbl;
	Dbl = pDbl.getValueDbl();
	cout << "copy:     " << Dbl << "\n";
	cout << "property: " << pDbl.getValueDbl() << "\n";
	cout << "... change copy ...\n";
	Dbl = 1;
	cout << "copy:     " << Dbl << "\n";
	cout << "property: " << pDbl.getValueDbl() << "\n";

	// Str
	cout << "\n\n<<PropertyStr>>\n";
	cout << "\nREFERENCE CAN BE USED TO CHANGE THE PROPERTY\n";
	string &StrRef = pStr.getValueStr();
	cout << "reference: " << StrRef << "\n";
	cout << "property:  " << pStr.getValueStr() << "\n";
	cout << "... change reference ...\n";
	StrRef = "gas";
	cout << "reference: " << StrRef << "\n";
	cout << "property:  " << pStr.getValueStr() << "\n";

	cout << "\nCOPY CANNOT BE USED TO CHANGE THE PROPERTY\n";
	string Str;
	Str = pStr.getValueStr();
	cout << "copy:     " << Str << "\n";
	cout << "property: " << pStr.getValueStr() << "\n";
	cout << "... change copy ...\n";
	Str = "vas";
	cout << "copy:     " << Str << "\n";
	cout << "property: " << pStr.getValueStr() << "\n";

	// IntArray
	cout << "\n\n<<PropertyIntArray>>\n";
	cout << "\nREFERENCE CAN BE USED TO CHANGE THE PROPERTY\n";
	Array<int> &intArrayRef = pIntArray.getValueIntArray();
	cout << "reference: " << intArrayRef << "\n";
	cout << "property:  " << pIntArray.getValueIntArray() << "\n";
	cout << "... change reference ...\n";
	intArrayRef[0] = 0;
	cout << "reference: " << intArrayRef << "\n";
	cout << "property:  " << pIntArray.getValueIntArray() << "\n";

	cout << "\nCOPY CANNOT BE USED TO CHANGE THE PROPERTY\n";
	Array<int> intArray(0);
	intArray = pIntArray.getValueIntArray();
	cout << "copy:     " << intArray << "\n";
	cout << "property: " << pIntArray.getValueIntArray() << "\n";
	cout << "... change copy ...\n";
	intArray[1] = 1;
	cout << "copy:     " << intArray << "\n";
	cout << "property: " << pIntArray.getValueIntArray() << "\n";

	// StrArray
	cout << "\n\n<<PropertyStrArray>>\n";
	cout << "\nREFERENCE CAN BE USED TO CHANGE THE PROPERTY\n";
	Array<string> &strArrayRef = pStrArray.getValueStrArray();
	cout << "reference: " << strArrayRef << "\n";
	cout << "property:  " << pStrArray.getValueStrArray() << "\n";
	cout << "... change reference ...\n";
	strArrayRef[0] = "wow";
	cout << "reference: " << strArrayRef << "\n";
	cout << "property:  " << pStrArray.getValueStrArray() << "\n";

	cout << "\nCOPY CANNOT BE USED TO CHANGE THE PROPERTY\n";
	Array<string> strArray("nothing");
	strArray = pStrArray.getValueStrArray();
	cout << "copy:     " << strArray << "\n";
	cout << "property: " << pStrArray.getValueStrArray() << "\n";
	cout << "... change copy ...\n";
	strArray[1] = "wow_wow";
	cout << "copy:     " << strArray << "\n";
	cout << "property: " << pStrArray.getValueStrArray() << "\n";

}


//_____________________________________________________________________________
/**
 * Test array.
 */
void TestArray()
{
	// DOUBLE
	Array<double> arrayDbl(20.0);
	// getLast()
	try {
		printf("arrayDbl[last] = %lf\n",arrayDbl.getLast());
	} catch(Exception x) {
		x.print(cout);
	}

	// append(), set(), get()
	int i;
	for(i=0;i<10;i++) arrayDbl.append((double)i);
	arrayDbl.set(2,0.0);
	arrayDbl.set(4,0.0);
	arrayDbl.set(6,0.0);
	arrayDbl.set(20,0.0);
	for(i=0;i<22;i++) {
		try {
			printf("arrayDbl[%d] = %lf\n",i,arrayDbl.get(i));
		} catch(Exception x) {
			x.print(cout);
		}
	}
	// insert()
	arrayDbl.insert(1,Math::PI);
	arrayDbl.insert(40,Math::PI);
	for(i=0;i<arrayDbl.getSize();i++) {
		try {
			printf("arrayDbl[%d] = %lf\n",i,arrayDbl.get(i));
		} catch(Exception x) {
			x.print(cout);
		}
	}
	// remove()
	arrayDbl.remove(1);
	arrayDbl.remove(0);
	arrayDbl.remove(1);
	arrayDbl.remove(2);
	arrayDbl.remove(3);
	arrayDbl.remove(-1);
	arrayDbl.remove(20);
	for(i=0;i<arrayDbl.getSize();i++) {
		printf("arrayDbl[%d] = %lf\n",i,arrayDbl[i]);
	}
	// setSize()
	for(i=0;i<arrayDbl.getSize();i++) {
		arrayDbl[i] = (double)i;
		printf("arrayDbl[%d] = %lf\n",i,arrayDbl[i]);
	}
	arrayDbl.setSize(20);
	for(i=0;i<arrayDbl.getSize();i++) {
		printf("arrayDbl[%d] = %lf\n",i,arrayDbl[i]);
	}
	arrayDbl.setSize(40);
	for(i=0;i<arrayDbl.getSize();i++) {
		printf("arrayDbl[%d] = %lf\n",i,arrayDbl[i]);
	}


	// ASSIGNMENT
	Array<double> *arrayCopy = new Array<double>(0.0);
	*arrayCopy = arrayDbl;
	for(i=0;i<arrayCopy->getSize();i++) {
		printf("arrayCopy[%d] = %lf\n",i,(*arrayCopy)[i]);
	}

	arrayDbl[20] = -40.0;
	for(i=0;i<arrayCopy->getSize();i++) {
		printf("arrayCopy[%d] = %lf\n",i,(*arrayCopy)[i]);
	}

	delete arrayCopy;

	// COPY CONSTRUCTOR
	arrayCopy = new Array<double>(arrayDbl);
	for(i=0;i<arrayCopy->getSize();i++) {
		printf("arrayCopy[%d] = %lf\n",i,(*arrayCopy)[i]);
	}

	arrayDbl.setSize(5);
	arrayDbl.trim();

	*arrayCopy = arrayDbl;
	arrayCopy->setSize(10);
	for(i=0;i<arrayCopy->getSize();i++) {
		printf("arrayCopy[%d] = %lf\n",i,(*arrayCopy)[i]);
	}
	printf("arrayCopy::_capacity = %d\n",arrayCopy->getCapacity());

	delete arrayCopy;
/*
	// BINARY SEARCH
	int n = 10000000;
	printf("\n\n");
	arrayDbl.setSize(0);
	for(i=0;i<=n;i++) arrayDbl.append((double)i + ((double)i)/Math::PI);
	//for(i=0;i<=20;i++) arrayDbl.set(i,20.0);
	//for(i=0;i<=n;i++) printf("array[%d] = %lf\n",i,arrayDbl[i]);
	double value;
	int index;
	for(i=0;i<=n;i = 2*i + 1) {
		value = 0.01 + (double)i;
		index = arrayDbl.searchBinary(value,true);
		if(index<0) {
			printf("Searching for %lf, no match found.\n",value);
		} else {
			printf("Searching for %lf, found array[%d]=%lf.\n",
				value,index,arrayDbl[index]);
		}
	}
*/
}


//_____________________________________________________________________________
/**
 * Test vector.
void TestVector()
{
	std::vector<int> y;
	printf("size = %d\n",y.size());
	printf("capacity = %d\n",y.capacity());

	y.reserve(13);
	printf("capacity = %d\n",y.capacity());

	y.clear();
	printf("size = %d\n",y.size());

	// PUSH VALUES ON TO VECTOR
	int i;
	for(i=0;i<21;i++) {
		y.push_back(i);
		printf("size = %d\n",y.size());
		printf("capacity = %d\n",y.capacity());
	}

	// PRINT VECTOR
	std::vector<int>::iterator iy;
	try {
		for(iy=y.begin(),i=0;i<y.size()+1;i++,iy++) {
			printf("y = %d\n",y[i]);
			printf("y = %d\n",*iy);
			printf("y = %d\n\n",y.at(i));
		}
	} catch(...) {
		printf("out of bounds\n");
	}
}
*/

//_____________________________________________________________________________
/**
 * Test exceptions.
 */
void TestExceptions()
{

	try {

		ThrowException();

	} catch(Exception x) {
		x.print(cout);
		return;
	} catch(int d) {
		printf("%d\n\n",d);
		return;
	} catch(...) {
		printf("Caught unknown exception type!\n\n");
	}

	printf("Proceding with execution.\n\n");

	return;
}

//_____________________________________________________________________________
/**
 * Throw an exception.
 */
void ThrowException()
{
	throw Exception("Test",__FILE__,__LINE__);
}


//_____________________________________________________________________________
/**
 * Test SIMM Utilities.
 */
void TestSIMMUtilities()
{
	// LOAD SIMM BONE FILE
	char *fileName = "C:\\cygwin\\home\\fca\\Projects\\TRex\\Bones\\femur.asc";
	int numVerts,numFaces,numConnects;
	double boundingBox[6];
	double *verts,*normals;
	int *counts,*connects;

	numConnects = SIMMUtilities::LoadBone(fileName,boundingBox,numVerts,verts,normals,
		numFaces,counts,connects);

	printf("numConnects = %d\n",numConnects);
}


//_____________________________________________________________________________
/**
 * Test Storage.
 */
void TestStorage()
{
	// CONSTRUCT STORAGE INSTANCE FROM FILE
	Storage store("test1.sto");

	// INTEGRATE THE STATES
	double ti = store.getFirstTime();
	double tf = store.getLastTime();
	Storage *integStore = store.integrate();

	// OPERATIONS
	Storage store2(store);
	store.add(&store2);
	store.divide(2.0);
	store.print("test2.sto",0.01);

	// WRITE WITH DIFFERENT OUTPUT
	IO::SetPrecision(3);
	IO::SetScientific(true);
	store.print("test2.sto",0.01);
	IO::SetPrecision(8);
	IO::SetScientific(false);
	integStore->print("test3.sto");

	delete integStore;
}

//_____________________________________________________________________________
/**
 * Test GCVSplineSet.
 */
void TestGCVSplineSet()
{
	// CONSTRUCT STORAGE
	// column 1 = PI
	// column 2 = x
	// column 3 = x*x
	// column 4 = x*x*x
	// column 5 = sin(x)
	Storage *store = new Storage(1000,"test");
	store->setDescription("\n\nFile for testing an GCVSplineSet.\nTADA\n\n");
	store->setColumnLabels("time\tconstant\tx\tx*x\tx*x*x\tsin(x)");
	double minX = -10.0;
	double maxX = 10.0;
	double aDX = 0.2;
	double x,y[5];
	for(x=minX;x<maxX;x+=aDX) {
		y[0] = Math::PI;
		y[1] = x;
		y[2] = x*x;
		y[3] = x*x*x;
		y[4] = sin(x);
		store->append(x,5,y);
	}
	store->print("gcv.sto");

	// CONSTRUCT GCVSplineSet
	int degree = 5;
	GCVSplineSet *splineSet = new GCVSplineSet(degree,store);

	// SERIALIZE
	splineSet->print("splineSet.fcn");

	// RECONSTRUCT STORAGE
	double dx = 0.0133;
	Storage *store0 = splineSet->constructStorage(0);
	store0->print("gcv0.sto");
	Storage *store1 = splineSet->constructStorage(1,dx);
	store1->print("gcv1.sto");
	Storage *store2 = splineSet->constructStorage(2,dx);
	store2->print("gcv2.sto");
	Storage *store3 = splineSet->constructStorage(3,dx);
	store3->print("gcv3.sto");
	Storage *store4 = splineSet->constructStorage(4,dx);
	store4->print("gcv4.sto");

	// DESERIALIZE
	try {
		GCVSplineSet splineSet2("./splineSet.fcn");
		splineSet2.print("./splineSet2.fcn");
	} catch(Exception x) {
		x.print(cout);
	}

	// CLEANUP
	delete splineSet;
	delete store;
	delete store0;
	delete store1;
	delete store2;
	delete store3;
	delete store4;

}
//_____________________________________________________________________________
/**
 * Test XML functionality.
void TestXML()
{
	// CREATE DEFAULT XML OBJECT
	//rdXML *xmlA = new rdXML();

	// GET DOCUMENT
	//IDOM_Document *docA = xmlA->getDocument();
	//printf("docA = %s\n",docA);


	// IO SETTINGS
	IO::SetPrecision(3);
	IO::SetDigitsPad(-1);

	// CREATE XML DOCUMENT FROM FILE
	//XMLDocument *doc = new XMLDocument("trex_coords.xml");
	//doc->print("trex_coords2.xml");

	// TRY MAKING A COPY
	//XMLDocument docCopy(*doc);

	// DELETE ORIGINAL DOCUMENT
	//delete doc;

	// PRINT COPIED DOCUMENT
	//docCopy.print("trex_coords3.xml");


	// CREATE A GRAPHICS MODEL FROM FILE
 	//rdGModel *gModel = new rdGModel("trex_coords.xml");

	// COMPUTE NEW BODY FRAMES
	rdGBodies *bodies = gModel->getBodies();
	int i;
	rdGBody *bod;
	for(i=0;i<bodies->getFirstEmpty();i++) {
		bod = bodies->getBody(i);
		if(bod==NULL) continue;
		bod->computeNewFrame();
		bod->estimateInertialProperties();
	}
	gModel->updateNode();
	gModel->print("trex_coords_out.xml");

	// GENERATE BONE FILES
	bodies->printBoneFiles();

	// CLEANUP
	delete gModel;
}
*/


//_____________________________________________________________________________
/**
 * Test geometry classes.
 */
void TestGeometry()
{
	printf("\n\nTestGeometry:\n");
	double o[] = { 0.0, 0.0, 0.0 };
	double pLine[] = { 1.0, 1.0, 1.0 };
	double dirLine[] = { 0.3, -0.77, 1.0 };
	double pPlane[] = { 0.0, 0.0, 0.0 };
	double x[] = { 1.0, 0.0, 0.0 };
	double y[] = { 0.0, 1.0, 0.0 };
	double z[] = { 0.0, 0.0, 1.0 };
	double xyz[] = { 1.0, 1.0, 1.0 };

	// CREATE A LINE
	Line line(pLine[0],pLine[1],pLine[2],dirLine);

	// CREATE A PLANE
	Plane plane(pPlane[0],pPlane[1],pPlane[2],z);

	// COMPUTE INTERSECTION
	double inter[3];
	int kind = plane.computeIntersection(&line,inter);

	// RESULTS
	if(kind==-1) {
		printf("Line and plane do not intersect.\n");

	} else if(kind==0) {
		printf("Line and plane intersect at %le,%le,%le\n",
			inter[0],inter[1],inter[2]);

	} else if(kind==1) {
		printf("Line and plane are coincident.  Line zero = %le,%le,%le\n",
			inter[0],inter[1],inter[2]);
	}
}
