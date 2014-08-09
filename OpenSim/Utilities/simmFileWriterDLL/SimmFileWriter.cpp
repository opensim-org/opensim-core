// SimmFileWriter.cpp
// Author: Peter Loan
/* Copyright (c)  2005, Stanford University and Peter Loan.
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <sstream>
#include "SimmFileWriter.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "SimbodySimmModel.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmFileWriter::SimmFileWriter() :
    _model(NULL),
    _simbodySimmModel(NULL)
{
}

//_____________________________________________________________________________
/**
 * Constructor taking a model pointer
 */
SimmFileWriter::SimmFileWriter(const Model& aModel) :
    _model(NULL),
    _simbodySimmModel(NULL)
{
    _model = &aModel;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmFileWriter::~SimmFileWriter()
{
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Write a SIMM joint file.
 *
 * @param aFileName name of joint file to write.
 * @return Whether or not file writing was successful.
 */
bool SimmFileWriter::writeJointFile(const string& aFileName)
{
    if (!_model)
        return false;

    if (!_simbodySimmModel)
        _simbodySimmModel = new SimbodySimmModel(_model);

    return _simbodySimmModel->writeJointFile(aFileName);
}

//_____________________________________________________________________________
/**
 * Write a SIMM muscle file.
 *
 * @param aFileName name of muscle file to write.
 * @return Whether or not file writing was successful.
 */
bool SimmFileWriter::writeMuscleFile(const string& aFileName)
{
    if (!_model)
        return false;

    if (!_simbodySimmModel)
        _simbodySimmModel = new SimbodySimmModel(_model);

    return _simbodySimmModel->writeMuscleFile(aFileName);
}
