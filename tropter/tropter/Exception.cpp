
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
