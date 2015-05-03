#ifndef OPENSIM_FILE_ADAPTER_H_
#define OPENSIM_FILE_ADAPTER_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  FileAdapter.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 * Contributors(s): Michael Sherman                                           *
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
* This file defines an abstract FileAdapter class, which implements the 
  OpenSim::DataAdpater interface for reading or writing data files.
*/
#include "DataAdapter.h"
#include <vector>
#include <string>
#include <fstream>

namespace OpenSim {
//=============================================================================
//=============================================================================
/** FileAdapter class for constructing DataAdapters that specifically access
    data sources that are files. It provides utilities for resolving paths and
    format parsing.
    Concrete classes handle the individual formats and specific DataTypes. */
class FileAdapter : public DataAdapter {

public:
    virtual ~FileAdapter() {}

    static FileAdapter* createAdapter(const std::string& idenitfier) {
        return dynamic_cast<FileAdapter*>(DataAdapter::createAdapter(idenitfier));
    }

    void setFilename(const std::string& filename) {
        closeDataSource();
        _filename = filename;
    }
    const std::string& getFilename() const { return _filename;  }

    /** Handy utility function for finding the file extension (the part after
    the dot) from a given file name. Returns an empty string if there is no
    dot or if the file name ends in a dot. **/
    static std::string findExtension(const std::string& fileName) {
        size_t found = fileName.find_last_of('.');
        return found == std::string::npos ? std::string()
            : fileName.substr(found + 1);
    }

    /** Convenient utility for chopping up a string into tokens separated
    by a specified delimiter character. Adjacent delimiters are illegal. **/
    static void tokenize(const std::string& s, char delim,
        Array<std::string>& tokens) {
        size_t start = 0, found = 0;
        while (start < s.size()) {
            found = s.find_first_of(delim, start);

            if (found == start) {
                throw Exception("FileAdapter::tokenize(): "
                    "empty token not allowed");
            }

            // hit the end of the string with no delimiter assume end;
            if (found > s.size()) {
                found = s.size();
            }

            // trim off any unnecessary whitespace
            std::string token = s.substr(start, found - start);
            //http://stackoverflow.com/questions/14233065/remove-whitespace-in-stdstring
            token.erase(std::remove_if(token.begin(), token.end(), ::isspace), 
                token.end());
            // append and return only stripped down token
            tokens.append(token);
            start = found + 1;
        }
        if (!tokens.size()) {
            throw Exception("FileAdapter::tokenize(): empty token at end of line");
        }
    }

protected:
    /** Only concrete FileAdpaters can default construct.
        Internal data stream(s) to file are NULL.*/
    FileAdapter() : DataAdapter() {}

    /** Only concrete FileAdpaters can construct from file name.*/
    FileAdapter(const std::string& filename) : DataAdapter(),
        _filename(filename) { 
            // handle path to the file
    }

private:
    // store the filename
    std::string _filename;

}; // DataAdapter

} // OpenSim namespace

#endif // OPENSIM_FILE_ADAPTER_H_