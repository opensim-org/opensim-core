// SimbodySimmBody.cpp
// Authors: Peter Loan
/*
 * Copyright (c)  2008, Stanford University. All rights reserved.
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
#include <iostream>
#include <string>
#include <math.h>
#include <float.h>
#include <time.h>

#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/Marker.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include "SimbodySimmBody.h"


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodySimmBody::~SimbodySimmBody()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimbodySimmBody::SimbodySimmBody()
{
}

//_____________________________________________________________________________
/**
 * Constructor from a Body.
 */
SimbodySimmBody::SimbodySimmBody(const OpenSim::Body* aBody, const string& aName)
{
    _name = aName;
    _body = aBody;
}

//_____________________________________________________________________________
/**
 * Write the Body to a [SIMM joint] file.
 */
void SimbodySimmBody::write(ofstream& aStream)
{
    aStream << "beginsegment " << _name << endl;

    if (_body != NULL) {
        aStream << "mass " << _body->getMass() << endl;

        SimTK::Vec3 massCenter = _body->getMassCenter();
        aStream << "masscenter " << massCenter[0] << " " << massCenter[1] << " " << massCenter[2] << endl;

        Mat33 inertia = _body->getInertia().toMat33();
        aStream << "inertia " << inertia[0][0] << " " << inertia[0][1] << " " << inertia[0][2] << endl;
        aStream << "        " << inertia[1][0] << " " << inertia[1][1] << " " << inertia[1][2] << endl;
        aStream << "        " << inertia[2][0] << " " << inertia[2][1] << " " << inertia[2][2] << endl;

        string fileName;
        for (int i = 0; i < _body->getDisplayer()->getNumGeometryFiles(); i++)
        {
            fileName = _body->getDisplayer()->getGeometryFileName(i);
            int dot = fileName.find_last_of(".");
            if (dot > 0)
                fileName.erase(dot, 4);
            fileName += ".asc";
            aStream << "bone " << fileName << endl;
        }

        const MarkerSet& markerSet = _body->getModel().getMarkerSet();
        for (int i = 0; i < markerSet.getSize(); i++)
        {
            const Marker& marker = markerSet.get(i);

            if (&marker.getBody() == _body)
            {
                // Write out log(_weight) + 1 as marker weight instead of _weight,
                // so that we won't have markers with radius 1000 if _weight=1000.
                // If _weight <= 1, outputWeight will be set to 1.
                //double outputWeight = (marker.getWeight() > 1.0) ? log(marker.getWeight()) + 1.0 : 1.0;
                // TODO: Got rid of weight property from markers for now...
                double outputWeight = 1;

                aStream << "marker " << marker.getName() << "\t" << marker.getOffset()[0] << " " <<
                        marker.getOffset()[1] << " " << marker.getOffset()[2] << " " << outputWeight;

                if (marker.getFixed())
                    aStream << " fixed" << endl;
                else
                    aStream << endl;
            }
        }

        SimTK::Vec3 scaleFactors;
        _body->getDisplayer()->getScaleFactors(scaleFactors);

        aStream << "scale " << scaleFactors[0] << " " << scaleFactors[1] << " " << scaleFactors[2] << endl;
    } else {
        aStream << "mass 0.000001" << endl;
        aStream << "masscenter 0.0 0.0 0.0" << endl;
        aStream << "inertia 0.0000001 0.0 0.0 0.0 0.0000001 0.0 0.0 0.0 0.0000001" << endl;
    }

    aStream << "endsegment" << endl << endl;
}
