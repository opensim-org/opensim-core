// Contributors: Ayman Habib
/* Copyright (c)  2006 Stanford University
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
#include <string.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>

using namespace OpenSim;
using namespace std;

int main(int argc,char **argv)
{
#ifndef STATIC_OSIM_LIBS
	LoadOpenSimLibrary("osimTools");
	LoadOpenSimLibrary("osimActuators");
	LoadOpenSimLibrary("osimAnalyses");
	LoadOpenSimLibrary("osimSimulation");
	LoadOpenSimLibrary("osimSimbodyEngine");
#endif

	int offset=0;
	if(argc>1 && string(argv[1])=="-offline") offset++;
	else IO::SetPrintOfflineDocuments(false);
	if(argc<1+offset+2) {
		std::cerr << "Not enough arguments: <INPUT_FILE> <OUTPUT_FILE>" << std::endl;
		exit(1);
	}
	string fileName = argv[offset+1];
	string outputFileName = argv[offset+2];
	Object *obj = Object::makeObjectFromFile(fileName);
	std::cout << fileName << " -> " << outputFileName;
	if(!obj) std::cout << " FAILED" << std::endl;
	else {
		std::cout << std::endl;
		//IO::SetGFormatForDoubleOutput(true);
		obj->copy()->print(outputFileName);
	}
}
