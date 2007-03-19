#ifndef __IKTaskSet_h__
#define __IKTaskSet_h__

#include "osimToolsDLL.h"
#include <OpenSim/Common/Set.h>
#include "IKTask.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * @authors Eran Guendelman
 * @version 1.0
 * - Added constructor from a file for use in GUI. -Ayman 02/20/07
 */

class OSIMTOOLS_API IKTaskSet : public Set<IKTask>
{
public:
	IKTaskSet() { setType("IKTaskSet"); }
	IKTaskSet(const IKTaskSet &aIKTaskSet) : Set<IKTask>(aIKTaskSet) { }
	IKTaskSet(const std::string &aFileName) : Set<IKTask>(aFileName) { }
#ifndef SWIG
	IKTaskSet& operator=(const IKTaskSet &aIKTaskSet) { Set<IKTask>::operator=(aIKTaskSet); return *this; }
#endif
//=============================================================================
};	// END of class IKTaskSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __IKTaskSet_h__
