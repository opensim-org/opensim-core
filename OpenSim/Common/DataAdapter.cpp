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
#include "DataTable.h"

namespace OpenSim {

bool DataAdapter::hasDataAccess() const
{
    return _hasDataAccess;
}

void  DataAdapter::setHasDataAccess(bool hasAccess)
{
    _hasDataAccess = hasAccess;
}

bool DataAdapter::openDataSource()
{
    _hasDataAccess = extendOpenDataSource();
    return _hasDataAccess;
}

bool DataAdapter::closeDataSource()
{
    bool closed = extendCloseDataSource();
    //closed successfully means no more access to the data
    _hasDataAccess = !closed;
    return closed;
}

std::ios_base::openmode DataAdapter::getAccessMode() const
{
    return _accessMode;
}

bool DataAdapter::isReadAccess() const {
    auto val = getAccessMode() & std::ios_base::in;
    bool ans = val ? true : false;
    return ans; 
}

bool DataAdapter::isWriteAccess() const {
    auto val = getAccessMode() & std::ios_base::out;
    bool ans = val ? true : false;
    return ans;
}

void DataAdapter::prepareForReading(AbstractDataTable& in)
{
    if (!hasDataAccess()) {
        openDataSource();
    }
    if (hasDataAccess() && isReadAccess()){
        extendPrepareForReading(in);
    }
    else {
        throw Exception("DataAdapter::prepareForReading()  "
            "DataAdapter does not have read access.");
    }
}

void DataAdapter::prepareForWriting(const AbstractDataTable& out)
{
    if (!hasDataAccess()) {
        openDataSource();
    }
    if (hasDataAccess() && isWriteAccess()){
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

bool DataAdapter::read() 
{
    if (isReadAccess()) {
        return extendRead();
    }
    throw Exception("DataAdapter does not support read access.");
}

bool DataAdapter::write()
{
    if (isWriteAccess()){
        return extendWrite();
    }
    
    throw Exception("DataAdapter does not support write access.");
}

void DataAdapter::extendPrepareForReading(AbstractDataTable& in) const
{
    throw Exception("DataAdapter::extendPrepareForReading() "
        "is not implemented.");
}

void DataAdapter::extendPrepareForWriting(const AbstractDataTable& out)
{
    throw Exception("DataAdapter::extendPrepareForWriting() "
        "is not implemented.");
}

bool DataAdapter::extendRead() const
{
    throw Exception("DataAdapter::extendRead() is not implemented.");
    return false;
}

bool DataAdapter::extendWrite()
{
    throw Exception("DataAdapter::extendWrite() is not implemented.");
    return false;
}

/* static */
std::map<std::string, std::unique_ptr<DataAdapter>, std::less<std::string> >
DataAdapter::_mapTypeNamesToAdapters;


DataAdapter* DataAdapter::createAdapter(const std::string& identifier)
{
    auto it = _mapTypeNamesToAdapters.find(identifier);
    if (it != _mapTypeNamesToAdapters.end()){
        return it->second->clone();
    }
    throw Exception("DataAdapter::createAdapter() adapter with identifier '"
        + identifier + "' could not be found.");
}

/* static */
void DataAdapter::registerDataAdapter(const std::string& identifier,
                                      const DataAdapter& adapter)
{
    auto it = _mapTypeNamesToAdapters.find(identifier);
    if (it == _mapTypeNamesToAdapters.end()){
        // add only if not previously registered
        _mapTypeNamesToAdapters[identifier] =
            std::unique_ptr<DataAdapter>(adapter.clone());
        return;
    }
    throw Exception("DataAdapter::registerDataAdapter() adapter for '"
        + identifier + "' already registered.");
}


} //namespace
//=============================================================================
//=============================================================================

