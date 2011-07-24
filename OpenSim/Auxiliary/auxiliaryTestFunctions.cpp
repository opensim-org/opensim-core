/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

// INCLUDE
#include <string>
#include <iostream>
#include <exception>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void ASSERT(bool cond, string file, int line, string message) {
	if (!cond) throw Exception(message, file, line);
}

void checkResultFiles(string resultsFile, string standardFile, Array<double> &tolerances, string file, int line, string message) {

	Array<string> columnLabels;
	Array<double> comparisons;
	int columns = compareResultFiles(resultsFile, standardFile, columnLabels, comparisons);

	for (int i = 1; i < columns; ++i) {
		cout << "column:    " << columnLabels[i] << endl;
		cout << "RMS error: " << comparisons[i] << endl;
		cout << "tolerance: " << tolerances[i] << endl << endl;
		ASSERT(comparisons[i] < tolerances[i], file, line, message);
	}
}

void checkCOM(string resultsFile, string body, SimTK::Vec3 &standardCOM, Array<double> &tolerances) {

	// compare the adjusted center of mass to OpenSim 1.9.1 values
	Model adjusted_model(resultsFile);
	const BodySet& bodies = adjusted_model.getBodySet();
	const Body& torso = bodies.get(bodies.getIndex(body));
	SimTK::Vec3 com;
	torso.getMassCenter(com);
	cout << "body:           " << body << endl;
	cout << "center of mass: (" << com[0] << ", " << com[1] << ", " << com[2] << ")\n";
	cout << "standard COM:   (" << standardCOM[0] << ", " << standardCOM[1] << ", " << standardCOM[2] << ")\n";
	cout << "tolerances:     (" << tolerances[0] << ", " << tolerances[1] << ", " << tolerances[2] << ")\n" << endl;
	for (int i = 0; i < 3; ++i)
		ASSERT_EQUAL(standardCOM[i], com[i], tolerances[i]);
}

int compareResultFiles(string resultsFile, string standardFile, Array<string> &columnLabels, Array<double> &comparisons) {

	Storage results(resultsFile);
	Storage standard(standardFile);

	columnLabels = results.getColumnLabels();
	comparisons.ensureCapacity(columnLabels.getSize());

	int i;
	for (i = 1; i < columnLabels.getSize(); ++i)
		comparisons[i] = results.compareColumnRMS(standard, columnLabels[i], 0.);

	return i;
}