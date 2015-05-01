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

#include "DataTable.h"
#include "osimCommonDLL.h"
#include "Object.h"
#include "Exception.h"
#include <string>
#include <memory>
#include <istream>

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
class OSIMCOMMON_API DataAdapter { //: public Object {
//    OpenSim_DECLARE_ABSTRACT_OBJECT(DataAdapter, Object);
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
    static void registerDataAdpater(const std::string& identifier,
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
//TODO    virtual DataTable& operator>>(DataTable& table) = 0;

    /** Direct DataTable data out through the DataAdapter. */
//TODO    virtual DataAdapter& operator<<(const DataTable& table) = 0;

    /** Determine read/write access of the adapter to data source.*/
    bool isReadAccess() const;
    bool isWriteAccess() const;

    /** Extract the meta data from the data source */
    const XMLDocument& getMetaData() const;
    /** If is write accessible, then permit the meta data to be updated.*/
    XMLDocument& updMetaData();

    /** Give the adapter an opportunity to prepare itself for reading. This may
        include accessing a file/stream, etc. Also allows the adapter prepare the
        DataTable for accepting data by setting column labels and the size of
        the DataTable prior to the reading in of rows of data. 
    @param[out] table  The DataTable to be filled by the DataAdapter. */
    void prepareForReading(AbstractDataTable& table);
    /** Give the adapter an opportunity to prepare itself for writing the
        the DataTable out. This may include updating meta data, opening a file,
        connecting to a device, etc... prior to writing out rows.
    @param[in] table  The DataTable to be written out by the DataAdapter. */
    void prepareForWriting(const AbstractDataTable& table);

    /** Read the column labels into the DataTable.
    @param[out] table  The DataTable to be filled by the DataAdapter. */
    void readColumnLabels(AbstractDataTable& table) const;
    /** Write the column labels from the DataTable out to the source.
    @param[in] table  The DataTable to be written out by the DataAdapter. */
    void writeColumnLabels(const AbstractDataTable& table);

    /** Read in rows of the DataTable from this adapter's data source.
    If a file this can be all the rows of data. If the source is a stream
    or a device, this may be a row in time or the last few buffered rows.
    @param[out] table DataTable containing the data read in by the DataAdapter.*/
    void readInTable(AbstractDataTable& table) const;

    /** Write out rows of the DataTableas to this adapter's data sink.
    If a file this can be all the rows of data. If the sink is a stream
    or a device, this may the row(s) up to the current time.
    @param[in] table DataTable containing the data to be written out by the
                     DataAdapter .*/
    void writeOutTable(const AbstractDataTable& table);

protected:
    /** Only derived DataAdapters can default construct the base */
    DataAdapter() {};

    /** Utility to set the access type that concrete adapters have to
    access the data. For example, read only Adapter have read = true,
    and write= false. */
    void setReadAccess(bool read);
    void setWriteAccess(bool write);

    /** Provide derived DataAdapters read access to the data of a DataTable */
    template<typename T>
    const SimTK::Matrix_<T>& getData(const AbstractDataTable& table) const {
        return DataTable_<T>::downcast(table)._data;
    }

    /** Provide the DataAdapter write access to the data of a DataTable */
    template<typename T>
    SimTK::Matrix_<T>& updData(AbstractDataTable& table) const {
        return DataTable_<T>::downcast(table)._data;
    }

    /** extend the preparations to the DataTable necessary for reading data */
    virtual void extendPrepareForReading(AbstractDataTable& dt);
    /** extend the preparations to the Adapter for writing out data */
    virtual void extendPrepareForWriting(const AbstractDataTable& dt);

    /** Implement the reading capability of the DataAdapter. DataAdapter reads
        the data from a concrete source one row at a time.
    @param[out] table DataTable being filled in by the DataAdapter
    @return success  (bool) whether or not the row was read from the source.*/
    virtual bool readNextRow(AbstractDataTable& table) const;
    /** Implement the writing capability of the DataAdapter to write out
    the data to a concrete source 
    @param[in] table DataTable being written out by the DataAdapter
    @param[in] rix   (size_t) index of the row to be written out
    @return success  (bool) whether or not the row was written to the sink.*/
    virtual bool writeOutRow(const AbstractDataTable& table, size_t rix);

    /** extend the reading capability of the DataAdapter to pull in
    the column labels for the data to be read in from a concrete source */
    virtual void extendReadColumnLabels(AbstractDataTable& dt) const;
    /** extend the writing capability of the DataAdapter to write out
    the column labels in the format of the concrete source */
    virtual void extendWriteColumnLabels(const AbstractDataTable& dt);

private:
    bool _isReadAccess;
    bool _isWriteAccess;

    // Registry of known concrete DataAdapters keyed by unique string
    // identifier (could be filename, extension, port, etc...) 
    static std::map<std::string, std::unique_ptr<DataAdapter>, 
        std::less<std::string> >  _mapTypeNamesToAdapters;
    
    XMLDocument _metaData;

//=============================================================================
};  // END of class DataAdapter


//=============================================================================
/**
* StreamWriter is a concrete class defining an interface for writing
* out the contents of a DataTable to a stream. It controls access to a stream.
*
* @author Ajay Seth
*/
template<typename DataType = SimTK::Real>
class StreamWriter : public DataAdapter {
//    OpenSim_DECLARE_CONCRETE_OBJECT_T(StreamWriter, DataType, DataAdapter);

public:
    StreamWriter() : DataAdapter() {
        setReadAccess(false);
        setWriteAccess(true);
    }

    virtual ~StreamWriter() {}

    StreamWriter(std::ostream& out) : _outputstream(&out) {
        setReadAccess(false);
        setWriteAccess(true);
    }

    StreamWriter* clone() const override { return new StreamWriter(*this); }

protected:

    void extendPrepareForWriting(const AbstractDataTable& dt) override {
        if (_outputstream.empty()){
            std::cout << "StreamWriter::extendPrepareForWriting() "
                << "No output stream specified.\n"
                << " Assuming std::cout." << std::endl;
            _outputstream = std::cout;
        }
        *_outputstream << dt.getDataTypeInfo().name() << "\n";
        *_outputstream << dt.getColumnLabels() << "\n";
    }

    /** extend the writing capability of the DataAdapter to write out
    the data to a std::ostream */
    bool writeOutRow(const AbstractDataTable& dt, size_t rix) override {
       *_outputstream << (DataTable_<DataType>::downcast(dt)).getRow(rix) << std::endl;
        return true;
    }

private:
    SimTK::ReferencePtr<std::ostream> _outputstream;

//=============================================================================
};  // END of class StreamWriter


//=============================================================================
/**
* StreamReader is a concrete class defining an interface for reading in
* the contents of a DataTable from a stream. It controls access to a stream.
*
* @author Ajay Seth
*/
template<typename DataType = SimTK::Real>
class StreamReader : public DataAdapter {
//    OpenSim_DECLARE_CONCRETE_OBJECT_T(StreamReader, DataType, DataAdapter);

public:
    StreamReader() : DataAdapter() {}
    virtual ~StreamReader() {}

    StreamReader(std::istream& in) : _inputstream(&in) {
        setReadAccess(true);
        setWriteAccess(false);
    }

    StreamReader* clone() const override { return new StreamReader(*this); }

    void extendPrepareForReading(AbstractDataTable& dt) override {
        if (_inputstream.empty()){
            throw std::runtime_error( 
                "StreamReader::extendPrepareForReading(): "
                "No input stream specified.\n"
                "Assuming std::cin." );
        }
    }

    /** extend the reading capability of the DataAdapter to write out
    the data to a std::istream */
    bool readNextRow(AbstractDataTable& dt) const override {
        std::string rowString;
        std::getline(*_inputstream, rowString);
        std::cout << "StreamReader::readNextRow() input stream contains:";
        std::cout << rowString << std::endl;
        RowVector_<DataType> row(int(dt.getNumCols()), DataType(0));
        DataTable_<DataType>::downcast(dt).appendRow(row);
        return !_inputstream->eof();
    }

private:
    SimTK::ReferencePtr<std::istream> _inputstream;
};

//typedef StreamReader<SimTK::Real> RealReader;
//typedef StreamReader<SimTK::Vec3> Vec3Reader;

} //namespace
//=============================================================================
//=============================================================================

#endif //OPENSIM_DATA_ADAPTER_H_
