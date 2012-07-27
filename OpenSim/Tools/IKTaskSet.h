#ifndef __IKTaskSet_h__
#define __IKTaskSet_h__
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

#include "osimToolsDLL.h"
#include <OpenSim/Common/Set.h>
#include "IKTask.h"
#include "IKMarkerTask.h"
#include <OpenSim/Simulation/MarkersReference.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * @authors Eran Guendelman
 * @version 1.0
 * - Added constructor from a file for use in GUI. -Ayman 02/20/07
 */

class OSIMTOOLS_API IKTaskSet : public Set<IKTask> {
OpenSim_DECLARE_CONCRETE_OBJECT(IKTaskSet, Set<IKTask>);

public:
	IKTaskSet() {}
	IKTaskSet(const IKTaskSet &aIKTaskSet) : Set<IKTask>(aIKTaskSet) { }
	IKTaskSet(const std::string &aFileName) : Set<IKTask>(aFileName) { }
	void createMarkerWeightSet(Set<MarkerWeight>& aWeights){
		for(int i=0; i< getSize(); i++){
			if(IKMarkerTask *nextTask = dynamic_cast<IKMarkerTask *>(&get(i))){
				aWeights.cloneAndAppend(*(new MarkerWeight(nextTask->getName(), nextTask->getWeight())));
			}
		}
	};
#ifndef SWIG
	IKTaskSet& operator=(const IKTaskSet &aIKTaskSet) { Set<IKTask>::operator=(aIKTaskSet); return *this; }
#endif
//=============================================================================
};	// END of class IKTaskSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __IKTaskSet_h__
