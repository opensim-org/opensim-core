/* -------------------------------------------------------------------------- *
 *                         OpenSim:  DataAdapter.cpp                          *
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

#include "DataAdapter.h"

namespace OpenSim {

bool DataAdapter::isReadAccess() const
{
    return _isReadAccess;
}

bool DataAdapter::isWriteAccess() const
{
    return _isWriteAccess;
}

const XMLDocument& DataAdapter::getMetaData() const
{
    return _metaData;
}

XMLDocument& DataAdapter::updMetaData()
{
    return _metaData;
}


void DataAdapter::prepareForReading(AbstractDataTable& in)
{
    if (isReadAccess()){
        extendPrepareForReading(in);
    }
    else {
        throw Exception("DataAdapter::prepareForReading()  "
            "DataAdapter does not have read access.");
    }
}

void DataAdapter::prepareForWriting(const AbstractDataTable& out)
{
    if (isWriteAccess()){
        if (out.getNumCols() != out.getColumnLabels().size()){
            throw Exception("DataAdapter::prepareForWriting():\n"
                "DataTable column labels inconsistent with the number of columns.");
        }
        extendPrepareForWriting(out);
    }
    else {
        throw Exception("DataAdapter::prepareForWriting()  "
            "DataAdapter does not have write access.");
    }
}

void DataAdapter::readColumnLabels(AbstractDataTable& table) const
{
    if (isReadAccess()){
        extendReadColumnLabels(table);
    }
    else {
        throw Exception("DataAdapter::readColumnLabels()  "
            "DataAdapter does not have read access.");
    }
}

void DataAdapter::writeColumnLabels(const AbstractDataTable& table)
{
    if (isWriteAccess()){
        extendWriteColumnLabels(table);
    }
    else {
        throw Exception("DataAdapter::writeColumnLabels()  "
            "DataAdapter does not have write access.");
    }
}


void DataAdapter::readInTable(AbstractDataTable& in) const
{
    if (isReadAccess()){
        size_t nrows = 0;
        while (readNextRow(in)) { ++nrows; }
        
        if (nrows > 0)
            return;

        throw Exception(
            "DataAdapter::readInTable() failed to read in DataTable." );
    }
    throw Exception("DataAdapter does not support read access.");
}

void DataAdapter::writeOutTable(const AbstractDataTable& out)
{
    if (isWriteAccess()){
        size_t nrow = 0;
        size_t numTableRows = out.getNumRows();
        for (nrow; nrow < numTableRows; ++nrow)
            writeOutRow(out, nrow);
        
        if (nrow == numTableRows)
            return;

        throw Exception(
            "DataAdapter::writeOutTable() failed to write the complete table.");
    }
    
    throw Exception("DataAdapter does not support write access.");
}

void DataAdapter::setReadAccess(bool read) { _isReadAccess = read; }

void DataAdapter::setWriteAccess(bool write) { _isWriteAccess = write; }

void DataAdapter::extendPrepareForReading(AbstractDataTable& in)
{
    throw Exception("DataAdapter::extendPrepareForReading() "
        "is not implemented.");
}

void DataAdapter::extendPrepareForWriting(const AbstractDataTable& out)
{
    throw Exception("DataAdapter::extendPrepareForWriting() "
        "is not implemented.");
}

bool DataAdapter::readNextRow(AbstractDataTable& in) const
{
    throw Exception("DataAdapter::extendReadInRows() "
        "is not implemented.");
    return false;
}

bool DataAdapter::writeOutRow(const AbstractDataTable& out, size_t rix)
{
    throw Exception("DataAdapter::extendWriteOutRows() "
        "is not implemented.");
    return false;
}

void DataAdapter::extendReadColumnLabels(AbstractDataTable& dt) const
{
    throw Exception("DataAdapter::extendReadColumnLabels() "
        "is not implemented.");
}

void DataAdapter::extendWriteColumnLabels(const AbstractDataTable& dt)
{
    throw Exception("DataAdapter::extendWriteolumnLabels() "
        "is not implemented.");
}

/* static */
std::map<std::string, std::unique_ptr<DataAdapter>, std::less<std::string> >
DataAdapter::_mapTypeNamesToAdapters;


DataAdapter* DataAdapter::createAdapter(const std::string& idenitfier) {
    std::map<std::string, std::unique_ptr<DataAdapter> >::const_iterator it
        = _mapTypeNamesToAdapters.find(idenitfier);
    if (it != _mapTypeNamesToAdapters.end()){
        return it->second->clone();
    }
    throw Exception("DataAdapter::createAdapter() adapter with identifier '"
        + idenitfier + "' could not be found.");
}


/* static */
void DataAdapter::registerDataAdpater(const std::string& sourceTypeName,
    const DataAdapter& adapter) {
    _mapTypeNamesToAdapters[sourceTypeName] =
        std::unique_ptr<DataAdapter>(adapter.clone());
}


} //namespace
//=============================================================================
//=============================================================================

