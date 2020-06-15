// ----------------------------------------------------------------------------
// tropter: Exception.cpp
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

#include "Exception.hpp"

using namespace tropter;

Exception::Exception(const std::string& msg) {
    set_message(msg);
}

Exception::Exception(const std::string& file, int line,
        const std::string& func)
        : m_file(file), m_line(line), m_func(func) {}

Exception::Exception(const std::string& file, int line,
        const std::string& func,
        const std::string& msg)
        : m_file(file), m_line(line), m_func(func) {
    set_message(msg);
}

void Exception::set_message(const std::string& msg) {
    m_msg = msg;
    m_what = "Tropter exception";
    if (!m_file.empty()) m_what += " thrown at " + file_name(m_file);
    if (m_line != -1) m_what += ":" + std::to_string(m_line);
    if (!m_func.empty()) m_what += " in " + m_func + "()";
    m_what += ":\n  " + m_msg + "\n";
}

const char* Exception::what() const noexcept {
    return m_what.c_str();
}

