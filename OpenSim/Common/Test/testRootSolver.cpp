// testTools.cpp
// Author:  Frank C. Anderson
#include <iostream>
#include <string>
#include <math.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Signal.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Object.h>
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
#include "ExampleVectorFunctionUncoupledNxN.h"




using namespace OpenSim;
using namespace std;


// DECLARATIONS

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

