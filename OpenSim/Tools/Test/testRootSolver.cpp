// testTools.cpp
// Author:  Frank C. Anderson
#include <iostream>
#include <string>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/Exception.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Signal.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/Line.h>
#include <OpenSim/Tools/Plane.h>
#include <OpenSim/Tools/GCVSplineSet.h>
#include <OpenSim/Tools/SIMMUtilities.h>
#include <OpenSim/Tools/Array.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/PropertyInt.h>
#include <OpenSim/Tools/PropertyIntArray.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyStrArray.h>
#include <OpenSim/Tools/PropertySet.h>
#include <OpenSim/Tools/RootSolver.h>
#include "ExampleVectorFunctionUncoupledNxN.h"




using namespace OpenSim;
using namespace std;


// DECLARATIONS

int TestRootSolver();
//_____________________________________________________________________________
/**
 * Test the rdTools library.
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

