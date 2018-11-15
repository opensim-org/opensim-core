#ifndef _StorageFactory_h_
#define _StorageFactory_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  StorageFactory.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* Factory to create Storage like objects, objects created depend on the extension 
 * passed to the createStorage method. Initially this is as follows
 *
 * ".sto", ".mot" -> Storage
 * ".trc" -> Storage
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
};  // END of class StorageFactory

}; //namespace
//=============================================================================
//=============================================================================

#endif //__StorageFactory_h__
