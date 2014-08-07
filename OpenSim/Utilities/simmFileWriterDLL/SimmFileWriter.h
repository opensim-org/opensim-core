#ifndef __SimmFileWriter_h__
#define __SimmFileWriter_h__

// SimmFileWriter.h
// Authors: Peter Loan
/*
 * Copyright (c)  2008, Stanford University and Peter Loan. All rights reserved.
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

// INCLUDE
#include <iostream>
#include <fstream>
#include <string>
#include "osimSimmFileWriterDLL.h"
#include "SimTKcommon.h"
#include <OpenSim/Common/Array.h>

#ifdef SWIG
#ifdef OSIMSIMMFILEWRITER_API
#undef OSIMSIMMFILEWRITER_API
#endif
#define OSIMSIMMFILEWRITER_API
#endif

namespace OpenSim {

class Model;
class SimbodySimmModel;

//=============================================================================
//=============================================================================
/**
 * A class for writing SIMM joint and muscle files for an OpenSim Model. Right
 * now all of the work is done by the SimbodySimmModel object, but someday there
 * could be additions to the SimmFileWriter interface, like setting directories
 * and other preferences.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMMFILEWRITER_API SimmFileWriter
{

//=============================================================================
// DATA
//=============================================================================
protected:
    const Model* _model;

    SimbodySimmModel* _simbodySimmModel;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    SimmFileWriter();
    SimmFileWriter(const Model& aModel);
    virtual ~SimmFileWriter();

    bool writeMuscleFile(const std::string& aFileName);
    bool writeJointFile(const std::string& aFileName);

//=============================================================================
};	// END of class SimmFileWriter
//=============================================================================
//=============================================================================
} // end of namespace OpenSim

#endif // __SimmFileWriter_h__


