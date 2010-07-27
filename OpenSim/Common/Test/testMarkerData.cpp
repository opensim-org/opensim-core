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
#include "OpenSim/Common/MarkerData.h"
#include <fstream>

using namespace OpenSim;
using namespace std;

#define ASSERT(cond) {if (!(cond)) throw exception();}

int main() {
	// Create a storge from a std file "std_storage.sto"
    try {
		//MarkerData md("TRCFileWithNANs.trc");
                MarkerData md = MarkerData("TRCFileWithNANS.trc");
		int rStartFrame=-1;
		int rEndFrame=-1;
		md.findFrameRange(0.0, 1.0, rStartFrame, rEndFrame);
		ASSERT(rStartFrame==1);
		ASSERT(rEndFrame==4);
		/*md.findFrameRange(0.004, 0.012, rStartFrame, rEndFrame);
		ASSERT(rStartFrame==1);
		ASSERT(rEndFrame==3);
		// ToBeTested void averageFrames(double aThreshold = -1.0, double aStartTime = -SimTK::Infinity, double aEndTime = SimTK::Infinity);
		ASSERT(md.getFileName()=="TRCFileWithNANs.trc");
		Storage storage;
		md.makeRdStorage(storage);*/
		//ASSERT(md.getUnits().getType()==Units(std::string mm("mm")).getType());
                //std::string mm("mm");
                Units lengthUnit = Units::Millimeters;
                ASSERT(md.getUnits().getType()==lengthUnit.getType())
		const Array<std::string>& markerNames = md.getMarkerNames();
		ASSERT(markerNames.getSize()==14);
		ASSERT(md.getMarkerIndex("toe")==0);
		ASSERT(md.getMarkerIndex("lASIS")==13);
		ASSERT(md.getMarkerIndex("NotFound")==-1);
		ASSERT(md.getNumFrames()==5);
		//ASSERT(md.getStartFrameTime()==0.0);
		//ASSERT(md.getLastFrameTime()==0.016);
		ASSERT(md.getDataRate()==250.);
		ASSERT(md.getCameraRate()==250.);
		//ToBeTested md.convertToUnits(Units(Units::Meters));
    }
    catch (...) {
        cout << "Failed" << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
