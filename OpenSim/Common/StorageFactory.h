#ifndef _StorageFactory_h_
#define _StorageFactory_h_
// StorageFactory.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005-2010, Stanford University. All rights reserved. 
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

/* Factory to create Storage like objects, objects created depend on the extension 
 * passed to the createStorage method. Initially this is as follows
 *
 * ".sto", ".mot" -> Storage
 * ".trc" -> SimmMotionData
 * 
 * There's support for users plugging in their own classes and having them handle arbitrary 
 * extensions for example ".c3d", ".trb" files
 *
 * Author: Ayman Habib 
 */

#include "osimCommonDLL.h"
//=============================================================================
//=============================================================================
/**
 *
 * @version 1.0
 * @author Ayman Habib
 */
namespace OpenSim { 

abstract class StorageCreator {
	virtual StorageInterface* createStorage(std::string& fileNameWithExtension)=0;
	virtual ~StorageCreator() {}
};
typedef std::map<std::string, OpenSim::StorageCreator*, std::less<std::string> > mapExtensionsToCreators;

// StorageCreator is a class that 
class OSIMCOMMON_API StorageFactory
{
//=============================================================================
// METHODS
//=============================================================================
private:
	mapExtensionsToCreators _mapExtensionsToCreators;
public:
	// make this constructor explicit so you don't get implicit casting of int to StorageFactory
	StorageFactory() {};

	~StorageFactory(){};

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// SIZE
	virtual StorageInterface* createStorage(std::string& fileNameWithExtension){
		std::string extension = fileNameWithExtension.substr(fileNameWithExtension. find_last_of("."));
		mapExtensionsToCreators::const_iterator find_Iter = _mapExtensionsToCreators.find(extension);
		Object* newObj=0;
		if (find_Iter != _mapExtensionsToCreators.end()){
			StorageCreator* defaultCreator = find_Iter->second;
			return defaultCreator->createStorage(fileNameWithExtension);
		}
		throw Exception("Don't know how to handle extension "+extension+" in StorageFactory");
	}
	static void registerStorageCreator(std::string& ext, StorageCreator* newCreator) {
		_mapExtensionsToCreators[ext]= newCreator;
	};  
//=============================================================================
};	// END of class StorageFactory

}; //namespace
//=============================================================================
//=============================================================================

#endif //__StorageFactory_h__
