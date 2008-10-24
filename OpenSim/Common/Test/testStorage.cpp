/*
* Copyright (c)  2008, Stanford University, All rights reserved. 
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

#include "OpenSim/Common/Storage.h"

using namespace OpenSim;
using namespace std;

#define ASSERT(cond) {if (!(cond)) throw exception();}

int main(int argc, char* argv[]) {
	// Create a storge from a std file "std_storage.sto"
   if (argc !=2)
   {
		cout << "incorrect number of parameters passed in to testStorage. Usage testStorage std_file" << endl;
      exit(1);
   }
   std::string stdLabels[] = {"time", "v1", "v2"};
    try {
		Storage* st = new Storage(argv[1]);
		// time[\t]v1[\t]v2
		// 1.[\t]	10.0[Space]20
		// 2.[\t\t] 20.0[\t]40
		ASSERT(st->getSize()==2);
		const Array<std::string> &lbls = st->getColumnLabels();
		ASSERT(lbls.getSize()==3);
		int i=0;
		for(i=0; i<lbls.getSize(); i++){
			ASSERT(lbls[i]==stdLabels[i]);
		}
		double val;
		for(i=0; i<st->getSize(); i++){
			StateVector& row = (*st->getStateVector(i));
			ASSERT(row.getTime()==i+1);
			ASSERT(row.getData()[0]==row.getTime()*10.0);
			row.getDataValue(0, val);
			ASSERT(val==row.getTime()*10.0);
			ASSERT(row.getData()[0]==row.getTime()*10.0);
			ASSERT(row.getData()[1]==row.getTime()*20.0);
		}

		delete st;
    }
    catch (...) {
        cout << "Failed" << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
