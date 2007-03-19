#ifndef _rdToolsTemplates_h_
#define _rdToolsTemplates_h_
// rdToolsTemplates.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// NOTES:
// This header file should not be included in any file that is a part of
// the rdTools library.  If it is included in files in the rdTools project,
// templates will be instantiated multiple times.
//
// Other projects, such as libraries other than rdTools or executables, should
// include this header file to import the template classes below.
//


// INCLUDES
#include "rdToolsDLL.h"
#include <string>
#include "Array.h"
#include "ArrayPtrs.h"
#include "NamedValueArray.h"
#include "Set.h"
#include "Material.h"


#ifdef WIN32

extern template class RDTOOLS_API Array<bool>;
extern template class RDTOOLS_API Array<int>;
extern template class RDTOOLS_API Array<double>;
extern template class RDTOOLS_API Array<std::string>;

extern template class RDTOOLS_API NamedValueArray<int>;
extern template class RDTOOLS_API NamedValueArray<double>;

extern template class RDTOOLS_API Set<Material>;

#endif  // WIN32


#endif  // __rdToolsTemplates_h__
