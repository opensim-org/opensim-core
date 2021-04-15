%include <OpenSim/JAM/osimJAMDLL.h>
%include <OpenSim/JAM/About.h>



namespace OpenSim {
    %ignore VTPFileAdapter::VTPFileAdapter(VTPFileAdapter &&);
    %ignore H5FileAdapter::H5FileAdapter(H5FileAdapter &&);
}



%shared_ptr(OpenSim::VTPFileAdapter)
%shared_ptr(OpenSim::H5FileAdapter)

%include <OpenSim/JAM/VTPFileAdapter.h>
%include <OpenSim/JAM/H5FileAdapter.h>

%include <OpenSim/JAM/COMAKSettings.h>
%template(SetCOMAKSecondaryCoordinates) OpenSim::Set<OpenSim::COMAKSecondaryCoordinate, OpenSim::Object>;
%include <OpenSim/JAM/COMAKSettingsSet.h>

//%include <OpenSim/JAM/JAMUtilities.h>
//%include <OpenSim/JAM/base64.h>
%include <OpenSim/JAM/COMAKInverseKinematicsTool.h>
//%include <OpenSim/JAM/COMAKTarget.h>
%include <OpenSim/JAM/COMAKTool.h>
%include <OpenSim/JAM/ForsimTool.h>
%include <OpenSim/JAM/JointMechanicsTool.h>


