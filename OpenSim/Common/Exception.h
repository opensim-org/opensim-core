#ifndef OPENSIM_EXCEPTION_H_
#define OPENSIM_EXCEPTION_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Exception.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "osimCommonDLL.h"
#include <string>

#ifdef _WIN32
#pragma warning(disable:4251) /*no DLL interface for type of member of exported class*/
#pragma warning(disable:4275) /*no DLL interface for base class of exported class*/
#endif

#ifdef SWIG
    #ifdef OSIMCOMMON_API
        #undef OSIMCOMMON_API
        #define OSIMCOMMON_API
    #endif
#endif

//=============================================================================
//=============================================================================

namespace OpenSim {

class Object;


/** @name Macros to throw OpenSim exceptions
The purpose of these macros is to aid with consistent message formatting,
include file/line/function information in all messages, and to make it easier
for developers to produce good messages.
@{                                                                            */
/**  
@relates OpenSim::Exception                                                   */
#define OPENSIM_THROW(EXCEPTION, ...)                                \
    throw EXCEPTION{__FILE__, __LINE__, __func__, __VA_ARGS__};

/** This macro checks the given condition and throws the given exception if the
condition is true. Here's an example that throws an exception if some result is
incorrect, and passes `result` and `5` to the constructor of the
`ResultIsIncorrect` exception:
@code
auto result = getSomeResult();
OPENSIM_THROW_IF(result != 5, ResultIsIncorrect, result, 5);
@endcode
@relates OpenSim::Exception                                                   */
// These macros also allow us to add more details (e.g. class name) later easily.
// Note -- Extra braces enclosing "if" are to avoid problems when these macros 
// are called within if-else statements like:
//           if(<some condition>)
//               OPENSIM_THROW_IF(<arguments>)
//           else
//               <some statements>
#define OPENSIM_THROW_IF(CONDITION, EXCEPTION, ...)                  \
    {                                                                \
    if(CONDITION)                                                    \
        OPENSIM_THROW(EXCEPTION, __VA_ARGS__)                        \
    }

/** Macro to throw from within an Object. This macro picks up implicit pointer
to the object and uses it to print information.                               */
#define OPENSIM_THROW_FRMOBJ(EXCEPTION, ...)                         \
    throw EXCEPTION{__FILE__, __LINE__, __func__, *this, __VA_ARGS__};

/** Macro to throw from within an Object if a condition evaluates to TRUE. This 
macro picks up implicit pointer to the object and uses it to print 
information.                                                                  */
#define OPENSIM_THROW_IF_FRMOBJ(CONDITION, EXCEPTION, ...)           \
    {                                                                \
    if(CONDITION)                                                    \
        OPENSIM_THROW_FRMOBJ(EXCEPTION, __VA_ARGS__)                 \
    }
/** @}                                                                        */


/**
 * A class for basic exception functionality.
 * \if developer
 * To create exception classes in OpenSim, use the following guidelines.
 * If the intention is to derive from an exception named, for example,
 * BaseException that is part of OpenSim, use the following blueprint:
 * \code{.cpp}
 *     class MyNewException : public BaseException {
 *     public:
 *         MyNewException(const std::string& file,
 *                        size_t line,
 *                        const std::string& func,
 *                        <more parameters as appropriate>) :
 *         BaseException(file, line, func) {
 *             std::string message = <create the desired message>;
 *             addMessage(message);
 *         }
 *     };
 * \endcode
 * Exception class manages the concatenation of error messages from all the 
 * derived classes. When creating new exceptions, remember to call addMessage()
 * as shown above if the exception class does have any error message.
 * \endif
 */
class OSIMCOMMON_API Exception  : public std::exception {

//=============================================================================
// DATA
//=============================================================================
private:
    /** A user set message for the exception. */
    std::string _msg;
    /** File in which the error occurred. */
    std::string _file;
    /** Line number at which the error occurred. */
    int _line;

//=============================================================================
// METHODS
//=============================================================================
public:
    // CONSTRUCTORS
    /** This constructor is for backward compatibility. Use the constructor
    taking file, line, func.                                                  */
    Exception(const std::string &aMsg="",
              const std::string &aFile="",
              int aLine=-1);

    /** Call this constructor from derived classes to add file, line and 
    function information to the error message. Use this when throwing
    Derived classes. Use OPENSIM_THROW_<> macros at throw sites.              */
    Exception(const std::string& file,
              size_t line,
              const std::string& func);

    /** Use this when you want to throw an Exception (with OPENSIM_THROW or
    OPENSIM_THROW_IF) and also provide a message.                             */
    Exception(const std::string& file,
              size_t line,
              const std::string& func,
              const std::string& msg);

    /** The message created by this constructor will contain the class name and
    instance name of the provided Object. Use this when throwing derived
    classes. Use OPENSIM_THROW_<> macros at throw sites.                      */
    Exception(const std::string& file,
              size_t line,
              const std::string& func,
              const Object& obj);

    /** The message created by this constructor will contain the class name and
    instance name of the provided Object, and also accepts a message. Use this
    when throwing Exception directly. Use OPENSIM_THROW_<> macros at throw 
    sites.                                                                    */
    Exception(const std::string& file,
              size_t line,
              const std::string& func,
              const Object& obj,
              const std::string& msg);

    virtual ~Exception() throw() {}

protected:
    /** Add to the error message that will be returned for the exception.     */
    void addMessage(const std::string& msg);

private:
    void setNull();

public:
    // SET AND GET
    void setMessage(const std::string &aMsg);
    const char* getMessage() const;

#ifndef SWIG
    // PRINT
    virtual void print(std::ostream &aOut) const;
#endif
    // override virtual function from std::exception
    const char* what() const noexcept override;

//=============================================================================
};  // END CLASS Exception


class InvalidArgument : public Exception {
public:
    InvalidArgument(const std::string& file,
                    size_t line,
                    const std::string& func,
                    const std::string& msg = "") :
        Exception(file, line, func) {
        std::string mesg = "Invalid Argument. " + msg;

        addMessage(mesg);
    }
};

class InvalidCall : public Exception {
public:
    InvalidCall(const std::string& file,
                size_t line,
                const std::string& func,
                const std::string& msg = "") :
        Exception(file, line, func) {
        std::string mesg = "Invalid Call. " + msg;

        addMessage(mesg);
    }
};

class InvalidTemplateArgument : public Exception {
public:
    InvalidTemplateArgument(const std::string& file,
                            size_t line,
                            const std::string& func,
                            const std::string& msg) :
        Exception(file, line, func) {
        std::string mesg = "Invalid Template argument. " + msg;

        addMessage(mesg);
    }
};

class IndexOutOfRange : public Exception {
public:
    IndexOutOfRange(const std::string& file,
                    size_t line,
                    const std::string& func,
                    size_t index,
                    size_t min, 
                    size_t max) :
        Exception(file, line, func) {
        std::string msg = "min = " + std::to_string(min);
        msg += " max = " + std::to_string(max);
        msg += " index = " + std::to_string(index);

        addMessage(msg);
    }
};

class KeyNotFound : public Exception {
public:
    KeyNotFound(const std::string& file,
                size_t line,
                const std::string& func,
                const std::string& key) :
        Exception(file, line, func) {
        std::string msg = "Key '" + key + "' not found.";

        addMessage(msg);
    }
};

class IOError : public Exception {
public:
    using Exception::Exception;
};

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_EXCEPTION_H_
