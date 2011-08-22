// testOptimizerExample.cpp

/* Copyright (c)  2009 Stanford University
 * Use of the OpenSim software in source form is permitted provided that the following
 * conditions are met:
 *   1. The software is used only for non-commercial research and education. It may not
 *     be used in relation to any commercial activity.
 *   2. The software is not distributed or redistributed.  Software distribution is allowed 
 *     only through https://simtk.org/home/opensim.
 *   3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
 *      presentations, or documents describing work in which OpenSim or derivatives are used.
 *   4. Credits to developers may not be removed from executables
 *     created from modifications of the source.
 *   5. Modifications of source code must retain the above copyright notice, this list of
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

// Author: Cassidy Kelly

//==============================================================================
//==============================================================================

#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace std;

int main()
{
	try {
		
		Storage result1("Arm26_randomSample_states_degrees.sto"), standard1("std_Arm26_randomSample_states_degrees.sto");
		result1.checkAgainstStandard(standard1, Array<double>(1000., 24), __FILE__, __LINE__, "Arm26 random sample states degrees failed");
		cout << "Arm26 random sample states degrees passed\n";
		
		Storage result2("Arm26_noActivation_states_degrees.sto"), standard2("std_Arm26_noActivation_states_degrees.sto");
		result2.checkAgainstStandard(standard2, Array<double>(1000., 24), __FILE__, __LINE__, "Arm26 no activation states degrees failed");
		cout << "Arm26 no activation states degrees passed\n";
		
		Storage result3("Arm26_bestSoFar_states_degrees.sto"), standard3("std_Arm26_bestSoFar_states_degrees.sto");
		result3.checkAgainstStandard(standard3, Array<double>(1000., 24), __FILE__, __LINE__, "Arm26 best so far states degrees failed");
		cout << "Arm26 best so far states degrees passed\n";
	}
	catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}