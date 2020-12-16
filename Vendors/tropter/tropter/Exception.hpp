#ifndef TROPTER_EXCEPTION_HPP
#define TROPTER_EXCEPTION_HPP
// ----------------------------------------------------------------------------
// tropter: Exception.hpp
// ----------------------------------------------------------------------------
// Copyright (c) 2017 tropter authors
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain a
// copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------

#include "Exception.h"
#include "utilities.h"

#include <sstream>

namespace tropter {

// Convert to types that can be printed with sprintf() (vsnprintf()).
template<typename T> struct make_pod_return { typedef T type; };

template<typename T>
inline typename make_pod_return<T>::type make_pod(const T& x) { return x; }

//template<>
//struct make_pod_return<adouble> { typedef double type; };
//
//template<>
//typename make_pod_return<adouble>::type make_pod(const adouble& x)
//{ return x.value(); }

template<> struct make_pod_return<std::string> { typedef const char* type; };
template<> inline
typename make_pod_return<std::string>::type make_pod(const std::string& x)
{   return x.c_str(); }

// This function was originally placed in this ".hpp" file because we were
// looking into using fmtlib, and we didn't want users to
// have to include fmt/format.h.
template<typename ...Types>
Exception::Exception(const std::string& file, int line,
        const std::string& func,
        const std::string& format_string, Types... args)
        : Exception(file, line, func) {
    //set_message(fmt::format(format_string, args...));
    set_message(format(format_string.c_str(), make_pod(args)...));
}

template <typename TActual, typename TExpected>
InvalidValue::InvalidValue(const std::string& file, int line,
        const std::string& func,
        const std::string& name,
        const TActual& actual, const TExpected& expected)
        : Exception(file, line, func) {
    std::stringstream ss;
    ss << "Invalid value for " << name << "\n" <<
            "  provided: " << actual << "\n" <<
            "  expected: " << expected << "\n";
    set_message(ss.str());
}

}; // namespace tropter


#endif // TROPTER_EXCEPTION_HPP
