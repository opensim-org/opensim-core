// testControllerExample.cpp

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
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

int main()
{
	try {
		Storage result1("tugOfWar_controls.sto"), standard1("std_tugOfWar_controls.sto");
		CHECK_STORAGE_AGAINST_STANDARD(result1, standard1, Array<double>(0.01, 24), __FILE__, __LINE__, "tugOfWar controls failed");
		cout << "\ntugOfWar controls passed\n" << endl;

		Storage result2("tugOfWar_states.sto"), standard2("std_tugOfWar_states.sto");
		CHECK_STORAGE_AGAINST_STANDARD(result2, standard2, Array<double>(0.01, 24), __FILE__, __LINE__, "tugOfWar states failed");
		cout << "\ntugOfWar states passed\n" << endl;
	}
	catch (const std::exception& e) {
        cout << e.what() << endl;
        return 1;
    }
    cout << "\nDone" << endl;
    return 0;
}