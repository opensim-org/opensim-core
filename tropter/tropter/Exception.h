#ifndef TROPTER_EXCEPTION_H
#define TROPTER_EXCEPTION_H

#include <string>
#include <exception>

namespace tropter {

#define TROPTER_THROW(...)                                                     \
    throw tropter::Exception(__FILE__, __LINE__, __func__, __VA_ARGS__);

#define TROPTER_THROW_EXC(EXCEPTION, ...)                                      \
    throw EXCEPTION(__FILE__, __LINE__, __func__, __VA_ARGS__);

#define TROPTER_THROW_IF(CONDITION, ...)                                       \
do {                                                                           \
    if ((CONDITION)) TROPTER_THROW(__VA_ARGS__);                               \
} while (false)

#define TROPTER_VALUECHECK(CONDITION, NAME, ACTUAL, EXPECTED)                  \
do {                                                                           \
    if (!(CONDITION)) TROPTER_THROW_EXC(InvalidValue, NAME, ACTUAL, EXPECTED); \
} while (false)


class Exception : public std::exception {
public:
    Exception() = default;
    Exception(const std::string& msg);
    Exception(const std::string& file, int line, const std::string& func);
    Exception(const std::string& file, int line, const std::string& func,
            const std::string& msg);

    template<typename ...Types>
    Exception(const std::string& file, int line, const std::string& func,
            const std::string& format_string, Types... args);

    const char* what() const noexcept override final;

protected:
    /// Assemble the text returned by what(). This should be called in the
    /// constructor of a derived type if the constructor does not pass `msg` to
    /// the base class constructor.
    void set_message(const std::string& msg);

private:
    /// Grab the last element of a file path.
    static std::string file_name(const std::string& filepath) {
        // Based on a similar function from Simbody.
        std::string::size_type pos = filepath.find_last_of("/\\");
        if (pos + 1 >= filepath.size()) pos = 0;
        return filepath.substr(pos + 1);
    }

    /// Format a string in the style of sprintf.
    std::string format(const char* format, ...) {
        // Get buffer size.
        va_list args;
        va_start(args, format);
        int bufsize = vsnprintf(nullptr, 0, format, args) + 1; // +1 for '\0'
        va_end(args);

        // Create formatted string.
        std::unique_ptr<char[]> buf(new char[bufsize]);
        va_start(args, format);
        vsnprintf(buf.get(), bufsize, format, args);
        va_end(args);
        return std::string(buf.get());
    }

    std::string m_file;
    int m_line = -1;
    std::string m_func;
    std::string m_msg;
    std::string m_what;

};

/// Thrown when a variable has an invalid value.
class InvalidValue : public Exception {
public:
    template <typename TActual, typename TExpected>
    InvalidValue(const std::string& file, int line, const std::string& func,
            const std::string& name,
            const TActual& actual, const TExpected& expected);
};

} //namespace tropter

#endif // TROPTER_EXCEPTION_H
