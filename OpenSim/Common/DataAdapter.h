#ifndef OPENSIM_DATA_ADAPTER_H_
#define OPENSIM_DATA_ADAPTER_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  DataAdapter.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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

/** @file
* This file defines the abstract DataAdapter class, which is used by OpenSim
* Data components to provide a consistent (DataTable) interface for creating
* a DataTable in memory for other OpenSim components to consume.
*/

#include "osimCommonDLL.h"
#include "Exception.h"
#include <SimTKcommon.h>
#include <string>
#include <memory>
#include <ios>
#include <map>

namespace OpenSim {

class AbstractDataTable;
//=============================================================================
//=============================================================================
/**
 * DataAdapter is an abstract class defining an interface for reading/writing
 * in/out the contents of a DataTable. It enables access to/from various data
 * sources/sinks such as: streams, files, databases and devices. The DataTable
 * is independent of the form and format of the data in/out of the source/sink.
 * Concrete classes handle the details (e.g. format, sequential access, etc...) 
 * associated with a particular data source/sink.
 *
 * The base DataAdapter contains a static registry to serve as a factory for 
 * concrete DataAdpaters, given a string identifier of the type of adapter.
 * The adapter knows the source format and data flow (read, write, both).
 * String identifiers can be associated with file formats according to known
 * file extensions.
 *
 * @author Ajay Seth
 */
class OSIMCOMMON_API DataAdapter { 
//=============================================================================
// METHODS
//=============================================================================
public:
    virtual ~DataAdapter(){};

    virtual DataAdapter* clone() const = 0;

    /** Creator of concrete DataAdapter(s) for the specified source type by its
        unique identifier (string). For example, for file based sources, a 
        component can acquire the necessary adapter instance to read the data
        by the file extension if the extension is used as its identifier.
    @param[in] identifier string used to uniquely identify the concrete adapter
               that processes data from/to sources/sinks of a particular type.
               For file adapters, .*/
    static DataAdapter* createAdapter(const std::string& identifier);

    /** Register a concrete DataAdapter by its unique string identifier.
        Registration permits access to the required concrete adapter by
        identifier lookup. As such, identifiers must be unique, but adapters may
        be registered with multiple identifiers. For example, a data file may 
        have multiple valid extensions (e.g. ".jpg: and ".jpeg") in which case
        both extensions would be valid identifiers for the same adapter. If an
        identifier is already in use an Exception is thrown. 
    @param[in] identifier string used to uniquely identify the required
               concrete adapter to process a source/sink. For file adapters,
               register unique file extension(s) in order for OpenSim to read/
               write different file formats.
    @param[in] adapter the concrete DataAdapter required to process the type
                       (format) of the data for the source/sink.*/
    static void registerDataAdapter(const std::string& identifier,
        const DataAdapter& adapter);

    /** Convenience construction of a DataTable from a data source identified
        by name and its type.
    @param[in] source           string identifying the source to be opened
    @param[in] sourceTypeName   string identifying the source type used to
                                determine which concrete adapter is used.
    @return DataTable           the resulting in memory data from the source
        */
    AbstractDataTable* createDataTable(const std::string& source,
        const std::string& sourceTypeName);

    /** Direct DataAdpater data flow into the provided DataTable. */
//TODO? DataTable& operator>>(DataTable& table);

    /** Direct DataTable data out through the DataAdapter. */
//TODO? DataAdapter& operator<<(const DataTable& table);

    /** Adapter has access to the data source for reading and/or writing? */
    bool hasDataAccess() const;

    /** Determine read/write access of the adapter to the data source.*/
    std::ios_base::openmode getAccessMode() const;

    /** Open the data source to enable access to read and/or writing of 
        data according to the access mode. If successful returns true.
        hasDataAccess() will also return true.*/
    bool openDataSource();

    /** Close the connection used to access the data source. 
        Returns true if successfully closed access to data. hasDataAccess() 
        also return false.*/
    bool closeDataSource();

    /** Give the adapter an opportunity to prepare itself for reading data
        into a DataTable. Adapter keeps a reference to the DataTable for
        all subsequent read access.
    @param[out] table  The DataTable to be filled by the DataAdapter. */
    void prepareForReading(AbstractDataTable& table);

    /** Give the adapter an opportunity to prepare itself for writing out a
        DataTable. Adapter keeps a reference to the DataTable for all
        subsequent write access.
    @param[in] table  The DataTable to be written out by the DataAdapter. */
    void prepareForWriting(const AbstractDataTable& table);

    /** Read in rows of the DataTable from this adapter's data source.
    If a file this can be all the rows of data. If the source is a stream
    or a device, this may be a row in time or the last few buffered rows.
    @return  done  returns true if there is no more data to be read in. */
    bool read();

    /** Write out rows of the DataTableas to this adapter's data sink.
    If a file this can be all the rows of data. If the sink is a stream
    or a device, this may the row(s) up to the current time.
    @return  done  returns true if the table is completely written out. */
    bool write();

    /** Convenience methods to interpret access mode */
    /** Return true if DataAdpater expects read access of the data source */
    bool isReadAccess() const;
    /** Return true if DataAdpater expects write access on the data source */
    bool isWriteAccess() const;

protected:
    /** Only derived DataAdapters can default construct the base */
    DataAdapter() {};

    /** Use this service to mark that the DataAdpater has successfully opened
        the data source. It should be known if the data source is open after
        prepareForReading/Writing. */
    void setHasDataAccess(bool hasAccess);

    /** Utility to set the access mode (flag) that concrete adapters have to
    in accessing the data source. For example, read only Adapter have 
    would only allow output operations.*/
    void setAccessMode(std::ios_base::openmode mode) {
        _accessMode = mode;
    }

    /** Provide derived DataAdapters read access to the data of a DataTable */
/*    template<typename T>
    const SimTK::Matrix_<T>& getData(const AbstractDataTable& table) const {
        return DataTable_<T>::downcast(table)._data;
    }
*/
    /** Provide the DataAdapter write access to the data of a DataTable */
/*    template<typename T>
    SimTK::Matrix_<T>& updData(AbstractDataTable& table) const {
        return DataTable_<T>::downcast(table)._data;
    }
*/
    /** Concrete DataApapters must open the DataSource according to their
        specified access requirements (e.g. Readers and Writers) */
    virtual bool extendOpenDataSource() = 0;
    /** Concrete DataApapters handle the hand-shaking to safely close
        the connection to the data source. */
    virtual bool extendCloseDataSource() = 0;

    /** extend the preparations to the DataTable necessary for reading data */
    virtual void extendPrepareForReading(AbstractDataTable& dt) const;
    /** extend the preparations to the Adapter for writing out data */
    virtual void extendPrepareForWriting(const AbstractDataTable& dt);

    /** Implement the reading capability of the DataAdapter. DataAdapter reads
        the data from a concrete source. It is completely up to concrete
        adapters on how the reading is done (e.g. line by line, while record
        at once, etc... This allows concrete adapters flexibility to meet the
        needs for access speed and/or intermittent data access.
    @return success  (bool) whether or not the row was read from the source.*/
    virtual bool extendRead() const;
    /** Implement the writing capability of the DataAdapter to write out
    the data to a concrete source 
    @param[in] rix   (size_t) index of the row to be written out
    @return success  (bool) whether or not the row was written to the sink.*/
    virtual bool extendWrite();

private:
    bool _hasDataAccess{ false };
    std::ios_base::openmode _accessMode{ 0 };

    // Registry of known concrete DataAdapters keyed by unique string
    // identifier (could be filename, extension, port, etc...) 
    static std::map<std::string, std::unique_ptr<DataAdapter>, 
        std::less<std::string> >  _mapTypeNamesToAdapters;

//=============================================================================
};  // END of class DataAdapter

} //namespace
//=============================================================================
//=============================================================================

#endif //OPENSIM_DATA_ADAPTER_H_
