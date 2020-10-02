/* -------------------------------------------------------------------------- *
 *                          OpenSim:  FileAdapter.h                           *
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

#ifndef OPENSIM_FILE_ADAPTER_H_
#define OPENSIM_FILE_ADAPTER_H_

/** @file
* This file defines an abstract FileAdapter class, which implements the 
  OpenSim::DataAdpater interface for reading or writing data files.
*/
#include "DataAdapter.h"

#include <vector>

namespace OpenSim {

class EmptyFileName : public InvalidArgument {
public:
    EmptyFileName(const std::string& file,
                  size_t line,
                  const std::string& func) :
        InvalidArgument(file, line, func) {
        std::string msg = "Filename is empty.";

        addMessage(msg);
    }
};

class FileDoesNotExist : public IOError {
public:
    FileDoesNotExist(const std::string& file,
                     size_t line,
                     const std::string& func,
                     const std::string filename) :
        IOError(file, line, func) {
        std::string msg = "File '" + filename + "' does not exist.";

        addMessage(msg);
    }
};

class FileIsEmpty : public IOError {
public:
    FileIsEmpty(const std::string& file,
                size_t line,
                const std::string& func,
                const std::string& filename) :
        IOError(file, line, func) {
        std::string msg = "File '" + filename + "' is empty.";

        addMessage(msg);
    }
};

class FileExtensionNotFound : public InvalidArgument {
public:
    FileExtensionNotFound(const std::string& file,
                          size_t line,
                          const std::string& func,
                          const std::string& filename) :
        InvalidArgument(file, line, func) {
        std::string msg = "Error inferring extension for file '";
        msg += filename + "'.";

        addMessage(msg);
    }
};

class UnexpectedColumnLabel : public IOError {
public:
    UnexpectedColumnLabel(const std::string& file,
                          size_t line,
                          const std::string& func,
                          const std::string& filename,
                          const std::string& expected,
                          const std::string& received) :
        IOError(file, line, func) {
        std::string msg = "Error reading column labels in file '" + filename;
        msg += "'. Unexpected column label. ";
        msg += "Expected = " + expected + ". ";
        msg += "Received = " + received + ". ";

        addMessage(msg);
    }
};

class RowLengthMismatch : public IOError {
public:
    RowLengthMismatch(const std::string& file,
                      size_t line,
                      const std::string& func,
                      const std::string& filename,
                      size_t line_num,
                      size_t expected,
                      size_t received) :
        IOError(file, line, func) {
        std::string msg = "Error reading rows in file '" + filename + "'. ";
        msg += "Unexpected number of columns in line ";
        msg += std::to_string(line_num) + ". ";
        msg += "Expected = " + std::to_string(expected) + ". ";
        msg += "Received = " + std::to_string(received) + ". ";

        addMessage(msg);
    }
};

class NoTableFound : public InvalidArgument {
public:
    NoTableFound(const std::string& file,
                 size_t line,
                 const std::string& func) :
        InvalidArgument(file, line, func) {
        std::string msg = "No table to write.";

        addMessage(msg);
    }
};

class KeyMissing : public InvalidArgument {
public:
    KeyMissing(const std::string& file,
               size_t line,
               const std::string& func,
               const std::string& key) :
        InvalidArgument(file, line, func) {
        std::string msg = "Key '" + key + "' missing.";

        addMessage(msg);
    }
};

class IncorrectTableType : public InvalidArgument {
public:
    IncorrectTableType(const std::string& file,
                       size_t line,
                       const std::string& func,
                       const std::string& message = "") :
        InvalidArgument(file, line, func) {
        std::string msg = "Incorrect Table type.";
        if(!message.empty())
            msg += " " + message;

        addMessage(msg);
    }
};

class TableMissingHeader : public Exception {
public:
    TableMissingHeader(const std::string& file,
                       size_t line,
                       const std::string& func) :
        Exception(file, line, func) {
        std::string msg = "Table does not have metadata for 'header'.";

        addMessage(msg);
    }
};

/** FileAdapter is a DataAdapter that reads and writes files with methods
read and writeFile respectively. The read method is implemented in the base class and it
calls the virtual extendRead method implemented by format specific subclasses. 
Typically you don't need to call read explicitly if reading one DataTable from file, instead use 
a constructor of the table from the specific file. e.g. 
\code{.cpp}
TimeSeriesTableVec3 table(filename);
 \endcode
 */
class OSIMCOMMON_API FileAdapter : public DataAdapter {
public:
    FileAdapter()                              = default;
    FileAdapter(const FileAdapter&)            = default;
    FileAdapter(FileAdapter&&)                 = default;
    FileAdapter& operator=(const FileAdapter&) = default;
    FileAdapter& operator=(FileAdapter&&)      = default;
    virtual ~FileAdapter()                     = default;

    /** Write a collection of tables to the given file. Different file formats
    require different number/type of tables. See specific adapter's 
    documentation to see what is required.                                    */
    static void writeFile(const InputTables& tables, 
                          const std::string& fileName);

    /** Find the extension from a filename.                                   */
    static
    std::string findExtension(const std::string& filename);

    /** Get the next line from the stream and tokenize/split the line using
    the given delimiters.                                                     */
    static std::vector<std::string> getNextLine(std::istream& stream,
        const std::string& delims);
   
    /** Tokenize/split a given string using the given delimiters. The delimiters
    are each required to be one character and the string is split if/when any 
    of those characters are found. For example, a delimiter string " \t" 
    specifies that either a space or a tab can act as the delimiter.          */
    static std::vector<std::string> tokenize(const std::string& str, 
                                      const std::string& delims);
    /** Create a concerte FileAdapter based on the extension of the passed in file and return it.
     This serves as a Factory of FileAdapters so clients don't need to know specific concrete 
     subclasses, as long as the generic base class read interface is used */
    static std::shared_ptr<DataAdapter> createAdapterFromExtension(const std::string& fileName);
};

} // OpenSim namespace

#endif // OPENSIM_FILE_ADAPTER_H_
