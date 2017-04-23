/* -------------------------------------------------------------------------- *
 *                           SimbodySimmBody.cpp                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, P2C HD065690, U54 EB020405)   *
 * and by DARPA through the Warrior Web program.                              *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <fstream>
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
      /*
      for (int i = 0; i < _body->getDisplayer()->getNumGeometryFiles(); i++)
      {
         fileName = _body->getDisplayer()->getGeometryFileName(i);
         int dot = fileName.find_last_of(".");
         if (dot > 0)
            fileName.erase(dot, 4);
         fileName += ".asc";
         aStream << "bone " << fileName << endl;
      }*/

      const MarkerSet& markerSet = _body->getModel().getMarkerSet();
      for (int i = 0; i < markerSet.getSize(); i++)
      {
         const Marker& marker = markerSet.get(i);
         const Body* refBody = dynamic_cast<const Body*>(&marker.getParentFrame());
         if (refBody && ( refBody == _body))
         {
            // Write out log(_weight) + 1 as marker weight instead of _weight,
            // so that we won't have markers with radius 1000 if _weight=1000.
            // If _weight <= 1, outputWeight will be set to 1.
            //double outputWeight = (marker.getWeight() > 1.0) ? log(marker.getWeight()) + 1.0 : 1.0;
            // TODO: Got rid of weight property from markers for now...
            double outputWeight = 1;

            aStream << "marker " << marker.getName() << "\t" << marker.get_location()[0] << " " <<
                marker.get_location()[1] << " " << marker.get_location()[2] << " " << outputWeight;

            if (false/*marker.getFixed()*/)
               aStream << " fixed" << endl;
            else
               aStream << endl;
         }
      }

      SimTK::Vec3 scaleFactors;
     // _body->getDisplayer()->getScaleFactors(scaleFactors);

      aStream << "scale " << scaleFactors[0] << " " << scaleFactors[1] << " " << scaleFactors[2] << endl;
   } else {
      aStream << "mass 0.000001" << endl;
      aStream << "masscenter 0.0 0.0 0.0" << endl;
      aStream << "inertia 0.0000001 0.0 0.0 0.0 0.0000001 0.0 0.0 0.0 0.0000001" << endl;
   }

    aStream << "endsegment" << endl << endl;
}
