// testTools.cpp
// Author:  Frank C. Anderson
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




using namespace OpenSim;
using namespace std;


//_____________________________________________________________________________
/**
 * Test the motion, storage file read/write library.
 */
int main(int argc, char* argv[])
{
	if (argc < 1) {
		std::cout << "Use at least one motion or storage file to parse" << endl;
		exit(-1);
	} 

	std::string fileName(argv[1]);
	Storage storage1 = Storage(fileName);
	std::cout << "Storage file:" << fileName << endl;
	std::cout << "Number of Rows:" << storage1.getSize() << endl;
	std::cout << "Number of Columns excluding time:" << storage1.getNumColumns() << endl;
	std::cout << "Use SIMM header:" << storage1.getWriteSIMMHeader() << endl;
	std::cout << "Number of Column Headers:" << storage1.getColumnLabels().getSize() << endl;
	std::cout << "Column Headers:" << storage1.getColumnLabels() << endl;

}
