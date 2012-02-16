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

#include <fstream>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

int main() {
	// Create a storge from a std file "std_storage.sto"
    try {
		//MarkerData md("TRCFileWithNANs.trc");
        MarkerData md("TRCFileWithNANS.trc");
        //MarkerData md = MarkerData("TRCFileWithNANS.trc");
		int rStartFrame=-1;
		int rEndFrame=-1;
		md.findFrameRange(0.0, 1.0, rStartFrame, rEndFrame);
		ASSERT(rStartFrame==0);
		ASSERT(rEndFrame==4);
		md.findFrameRange(0.004, 0.012, rStartFrame, rEndFrame);
		ASSERT(rStartFrame==1);
		ASSERT(rEndFrame==3);
		// ToBeTested void averageFrames(double aThreshold = -1.0, double aStartTime = -SimTK::Infinity, double aEndTime = SimTK::Infinity);
		ASSERT(md.getFileName()=="TRCFileWithNANS.trc");
		Storage storage;
		md.makeRdStorage(storage);
		ASSERT(md.getUnits().getType()==Units(string("mm")).getType(), __FILE__, __LINE__);
		//std::string mm("mm");
		Units lengthUnit = Units::Millimeters;
		ASSERT(md.getUnits().getType()==lengthUnit.getType(), __FILE__, __LINE__);
		const Array<std::string>& markerNames = md.getMarkerNames();
		ASSERT(markerNames.getSize()==14, __FILE__, __LINE__);
		ASSERT(md.getMarkerIndex("toe")==0, __FILE__, __LINE__);
		ASSERT(md.getMarkerIndex("lASIS")==13, __FILE__, __LINE__);
		ASSERT(md.getMarkerIndex("NotFound")==-1, __FILE__, __LINE__);
		ASSERT(md.getNumFrames()==5, __FILE__, __LINE__);
		ASSERT(md.getStartFrameTime()==0.0, __FILE__, __LINE__);
		ASSERT(md.getLastFrameTime()==0.016, __FILE__, __LINE__);
		ASSERT(md.getDataRate()==250., __FILE__, __LINE__);
		ASSERT(md.getCameraRate()==250., __FILE__, __LINE__);
		//ToBeTested md.convertToUnits(Units(Units::Meters));

		MarkerData md2("Run_500 02.trc");
		double expectedData[] = {1006.513977, 1014.924316,-195.748917};
		const MarkerFrame& frame2 = md2.getFrame(1);
		ASSERT(frame2.getFrameTime()==.01, __FILE__, __LINE__);
		const SimTK::Array_<SimTK::Vec3>& markers = frame2.getMarkers();
		const SimTK::Vec3& m1 = markers[0];
		ASSERT(SimTK::isNaN(m1[0]), __FILE__, __LINE__);
		ASSERT(SimTK::isNaN(m1[1]), __FILE__, __LINE__);
		ASSERT(SimTK::isNaN(m1[2]), __FILE__, __LINE__);
		SimTK::Vec3 diff = (markers[1]-SimTK::Vec3(expectedData[0], expectedData[1], expectedData[2]));
		ASSERT(diff.norm() < 1e-7, __FILE__, __LINE__);

		MarkerData md3("chris_mri-prot-writing-trial-11-twosheet-right.trc");
		double expectedData3[] = {-1.52E-01,	2.45E-01,	-1.71E+00};
		const MarkerFrame& frame3 = md3.getFrame(0);
		const SimTK::Array_<SimTK::Vec3>& markers3 = frame3.getMarkers();
		const SimTK::Vec3& m31 = markers3[1];    
		SimTK::Vec3 diff3 = (markers3[1]-SimTK::Vec3(expectedData3));
		ASSERT(diff.norm() < 1e-7, __FILE__, __LINE__);
	}
    catch(const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
