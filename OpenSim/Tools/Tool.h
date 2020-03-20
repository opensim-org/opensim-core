#ifndef _Tool_h_
#define _Tool_h_
/* -------------------------------------------------------------------------- *
 *                              OpenSim:  Tool.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyObj.h>


namespace OpenSim { 


//=============================================================================
//=============================================================================
/**
 * A Tool is an OpenSim abstraction that encapsulates an analysis or series of
 * modeling and analysis steps. Its primary duty is to provide an interface
 * for use by the GUI or as a standalone command line executable. It includes
 * common methods for invoking the tool and performing routine I/O.
 * 
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMTOOLS_API Tool : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(Tool, Object);

//=============================================================================
// DATA
//=============================================================================
public:
    
    OpenSim_DECLARE_PROPERTY(results_directory, std::string,
            "Name of the directory where results are written. Be default this "
            "is the directory in which the setup file is be  executed.");

    //=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    
    /**
    * Destructor.
    */
    virtual ~Tool() {};
    
    /**
    * Default constructor.
    */
    Tool() { 
        constructProperties();
    };
    
    /**
    * Construct from file
    *
    * The object is constructed from the root element of the XML document.
    * The type of object is the tag name of the XML root element.
    *
    * @param aFileName File name of the document.
    * @param aUpdateFromXMLNode
    */
    Tool(const std::string &aFileName, bool aUpdateFromXMLNode = true):
        Object(aFileName, true) {
            constructProperties();
            updateFromXMLDocument();
        };
    
private:
    void constructProperties() { 
        constructProperty_results_directory("./");
    }

public:
    //--------------------------------------------------------------------------
    // INTERFACE
    //--------------------------------------------------------------------------
    /** The run() method of a Tool embodies what would be the main() routine 
        for a standalone program.  Therefore, any OpenSim main program can
        become a Tool executable from the GUI by making it a run() method for
        a new Tool.
        
        It is expected that the run() method be composed of a sequence of calls  
        to underlying computational and reporting objects and should 
    */
    virtual bool run() SWIG_DECLARE_EXCEPTION=0;

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    /** 
    * Get/set Results Directory, will replace with Property accessors eventually
    */
    const std::string& getResultsDir() const { return get_results_directory(); }
    void setResultsDir(const std::string& aString) { set_results_directory(aString); }

//=============================================================================
};  // END of class Tool

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Tool_h__


