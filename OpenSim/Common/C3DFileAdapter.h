/* -------------------------------------------------------------------------- *
 *                          OpenSim:  C3DFileAdapter.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#ifndef OPENSIM_C3D_FILE_ADAPTER_H_
#define OPENSIM_C3D_FILE_ADAPTER_H_

#if defined (WITH_EZC3D)

#include "FileAdapter.h"
#include "TimeSeriesTable.h"
#include "Event.h"

namespace OpenSim {

/** C3DFileAdapter reads a C3D file into markers and forces tables of type
TimeSeriesTableVec3. The markers table has each column labeled by its
corresponding marker name. For the forces table, the data are grouped
by sensor (force-plate #) in force, point and moment order, with the
respective *f#*, *p#* and *m#* column labels. C3DFileAdpater provides
options for expressing the force-plate measurements either as the
net force and moments expressed at the ForcePlateOrigin, the
CenterOfPressure, or the PointOfWrenchApplication. */
class OSIMCOMMON_API C3DFileAdapter : public FileAdapter {
public:
    typedef std::vector<Event>                         EventTable; 
    typedef std::map<std::string, std::shared_ptr<TimeSeriesTableVec3>> Tables;

    /** Enumerated list of locations in which read in forces are expressed.
        %Measurement from force plates can be expressed by the C3DFileAdapter
        either at the OriginOfForcePlate (the default), CenterOfPressure, or
        the PointOfWrenchApplication. You need to call  setLocationForForceExpression
        before invoking C3DFileAdapter::read(), otherwise the default location
        (OriginOfForcePlate) is assumed.

        In the case of the CenterOfPressure (COP), the underlying assumptions
        are that the ground plane (in which COP is defined) passes through the
        lab origin (0,0,0) with the Z-axis as its normal vector.

        The PointOfWrenchApplication (PWA) does not assume a plane of contact.
        The PWA is an equivalent wrench in the lab frame and computed according
        to Shimba 1984.
        Takeshi Shimba, An estimation of center of gravity from force platform
        data, Journal of Biomechanics, 17(1), pp53-60, 1984.

        <b>C++ example</b>
        \code{.cpp}
        C3DFileAdapter c3dFileAdapter;
        c3dFileAdapter.setLocationForForceExpression(C3DFileAdapter::ForceLocation::CenterOfPressure);
        auto tables  =  c3dFileAdapter.read("myData.c3d");
        auto markersTable = c3dFileAdapter.getMarkersTable(tables);
        auto forcesTable = c3dFileAdapter.getForcesTable(tables);
        \endcode

        <b>Python example</b>
        \code{.py}
        import opensim
        c3dFileAdapter = opensim.C3DFileAdapter()
        c3dFileAdapter.setLocationForForceExpression(opensim.C3DFileAdapter.ForceLocation_CenterOfPressure);
        tables = c3dFileAdapter.read("myData.c3d")
        markersTable = c3dFileAdapter.getMarkersTable(tables)
        forcesTable = c3dFileAdapter.getForcesTable(tables)
        \endcode

        <b>Java example</b>
        \code{.java}
        C3DFileAdapter c3dFileAdapter = new C3DFileAdapter();
        c3dFileAdapter.setLocationForForceExpression(C3DFileAdapter.ForceLocation.CenterOfPressure);
        tables = c3dFileAdapter.read("myData.c3d");
        TimeSeriesTableVec3 markersTable = c3dFileAdapter.getMarkersTable(tables);
        TimeSeriesTableVec3 forcesTable = c3dFileAdapter.getForcesTable(tables);

        \endcode

        <b>MATLAB example</b>
        \code{.m}
         c3dFileAdapter = C3DFileAdapter();
         c3dFileAdapter.setLocationForForceExpression(1);
         tables = c3dFileAdapter.read('myData.c3d');
         markersTable = c3dFileAdapter.getMarkersTable(tables);
         forcesTable = c3dFileAdapter.getForcesTable(tables);
        \endcode
    */
    enum class ForceLocation {
        OriginOfForcePlate       = 0,   ///< 0 : the origin of the force-plate
        CenterOfPressure         = 1,   ///< 1 : the center of pressure
        PointOfWrenchApplication = 2    ///< 2 : PWA as defined by Shimba, 1984
    };

    C3DFileAdapter* clone() const override;
    /**  C3DFileAdpater provides options for expressing the force-plate 
        measurements either as the net force and moments expressed at the 
        ForcePlateOrigin, the CenterOfPressure, or the 
        PointOfWrenchApplication.  This function sets the option. */
    void setLocationForForceExpression(const ForceLocation location) {
        _location = location;
    }
    /** Retrieve the option for location for force expression */
    ForceLocation getLocationForForceExpression() const {
        return _location;
    }

#ifndef SWIG
    static
    void write(const Tables& markerTable, const std::string& fileName);
#endif

    /** Retrieve the TimeSeriesTableVec3 of Markers */
    std::shared_ptr<TimeSeriesTableVec3> getMarkersTable(DataAdapter::OutputTables& tables) {
        std::shared_ptr<AbstractDataTable>& adt = tables.at("markers");
        return std::dynamic_pointer_cast<TimeSeriesTableVec3>(adt);
    }

    /** Retrieve the TimeSeriesTableVec3 of Forces */
     std::shared_ptr<TimeSeriesTableVec3> getForcesTable(DataAdapter::OutputTables& tables) {
        std::shared_ptr<AbstractDataTable>& adt = tables.at("forces");
        return std::dynamic_pointer_cast<TimeSeriesTableVec3>(adt);
    }

    static const std::string _markers;
    static const std::string _forces;


protected:
    OutputTables extendRead(const std::string& fileName) const override;

    void extendWrite(const InputTables& tables,
                     const std::string& fileName) const override;

private:
    static const std::unordered_map<std::string, std::size_t> _unit_index;

    ForceLocation _location{ ForceLocation::OriginOfForcePlate };

};

} // namespace OpenSim

#endif // WITH_EZC3D

#endif // OPENSIM_C3D_FILE_ADAPTER_H_
