/* -------------------------------------------------------------------------- *
 *                  OpenSim:  AbstractDataTable.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Authors:                                                                   *
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

#include "AbstractDataTable.h"

namespace OpenSim {

size_t AbstractDataTable::getNumRows() const {
    return implementGetNumRows();
}

size_t AbstractDataTable::getNumColumns() const {
    return implementGetNumColumns();
}

void 
AbstractDataTable::removeTableMetaDataKey(const std::string& key) {
    _tableMetaData.removeValueForKey(key);
}

std::vector<std::string> 
AbstractDataTable::getTableMetaDataKeys() const {
    return _tableMetaData.getKeys();
}

const AbstractDataTable::TableMetaData& 
AbstractDataTable::getTableMetaData() const {
    return _tableMetaData;
}

AbstractDataTable::TableMetaData& 
AbstractDataTable::updTableMetaData() {
    return _tableMetaData;
}

const AbstractDataTable::IndependentMetaData& 
AbstractDataTable::getIndependentMetaData() const {
    return _independentMetaData;
}

void 
AbstractDataTable::setIndependentMetaData(const IndependentMetaData& 
                                          independentMetaData) {
    _independentMetaData = independentMetaData;
    validateIndependentMetaData();
}

const AbstractDataTable::DependentsMetaData& 
AbstractDataTable::getDependentsMetaData() const {
    return _dependentsMetaData;
}

void 
AbstractDataTable::setDependentsMetaData(const DependentsMetaData& 
                                         dependentsMetaData) {
    _dependentsMetaData = dependentsMetaData;
    validateDependentsMetaData();
}

void
AbstractDataTable::removeDependentsMetaDataForKey(const std::string& key) {
    _dependentsMetaData.removeValueForKey(key);
}

bool
AbstractDataTable::hasColumnLabels() const {
    return _dependentsMetaData.hasKey("labels");
}

std::vector<std::string> 
AbstractDataTable::getColumnLabels() const {
    OPENSIM_THROW_IF(!hasColumnLabels(),
                     NoColumnLabels);

    const auto& absArray = 
        _dependentsMetaData.getValueArrayForKey("labels");
    std::vector<std::string> labels{};
    for(size_t i = 0; i < absArray.size(); ++i)
        labels.push_back(absArray[i].getValue<std::string>());

    return labels;
}

const std::string& 
AbstractDataTable::getColumnLabel(const size_t columnIndex) const {
    OPENSIM_THROW_IF(!hasColumnLabels(),
                     NoColumnLabels);

    const auto& labels = 
        _dependentsMetaData.getValueArrayForKey("labels");

    OPENSIM_THROW_IF(columnIndex >= labels.size(),
                     ColumnIndexOutOfRange,
                     columnIndex, 0, 
                     static_cast<unsigned>(labels.size() - 1));

    return labels[columnIndex].getValue<std::string>();
}

void 
AbstractDataTable::setColumnLabels(const std::initializer_list<std::string>&
                                   columnLabels) {
    setColumnLabels(columnLabels.begin(), columnLabels.end());
}

void 
AbstractDataTable::setColumnLabel(const size_t columnIndex,
                                  const std::string& columnLabel) {
    using namespace SimTK;
    using namespace std;

    OPENSIM_THROW_IF(!hasColumnLabels(),
                     NoColumnLabels);

    ValueArray<std::string> newLabels{};
    const auto& oldLabels = 
        _dependentsMetaData.getValueArrayForKey("labels");

    OPENSIM_THROW_IF(columnIndex >= oldLabels.size(),
                     ColumnIndexOutOfRange,
                     columnIndex, 0, 
                     static_cast<unsigned>(oldLabels.size() - 1));

    for(unsigned i = 0; i < oldLabels.size(); ++i) {
        if(i == columnIndex) {
            newLabels.upd().push_back(Value<string>(columnLabel));
        } else {
            auto value = Value<string>(oldLabels[i].getValue<string>());
            newLabels.upd().push_back(value);
        }
    }

    _dependentsMetaData.removeValueArrayForKey("labels");
    _dependentsMetaData.setValueArrayForKey("labels", newLabels);

    validateDependentsMetaData();
}

size_t 
AbstractDataTable::getColumnIndex(const std::string& columnLabel) const {
    OPENSIM_THROW_IF(!hasColumnLabels(),
                     NoColumnLabels);

    const auto& absArray = 
        _dependentsMetaData.getValueArrayForKey("labels");
    for(size_t i = 0; i < absArray.size(); ++i)
        if(absArray[i].getValue<std::string>() == columnLabel)
            return i;

    OPENSIM_THROW(KeyNotFound, columnLabel);
}

bool 
AbstractDataTable::hasColumn(const std::string& columnLabel) const {
    OPENSIM_THROW_IF(!hasColumnLabels(),
                     NoColumnLabels);

    const auto& absArray = 
        _dependentsMetaData.getValueArrayForKey("labels");
    for(size_t i = 0; i < absArray.size(); ++i)
        if(absArray[i].getValue<std::string>() == columnLabel)
            return true;

    return false;
}

bool 
AbstractDataTable::hasColumn(const size_t columnIndex) const {
    return columnIndex < getNumColumns();
}

void
AbstractDataTable::appendColumnLabel(const std::string& columnLabel) {
    auto& absArray = _dependentsMetaData.updValueArrayForKey("labels");
    auto& labels = static_cast<ValueArray<std::string>&>(absArray);
    labels.upd().push_back(SimTK::Value<std::string>{columnLabel});

    validateDependentsMetaData();
}

} // namespace OpenSim
