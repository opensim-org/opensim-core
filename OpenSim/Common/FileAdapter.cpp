#include "FileAdapter.h"

namespace OpenSim {

FileAdapter::OutputTables
FileAdapter::readFile(const std::string& fileName) {
    auto extension = findExtension(fileName);
    auto data_adapter = createAdapter(extension);
    auto& file_adapter = static_cast<FileAdapter&>(*data_adapter);
    return file_adapter.extendRead(fileName);
}

void 
FileAdapter::writeFile(const InputTables& tables, 
                       const std::string& fileName) {
    auto extension = findExtension(fileName);
    auto data_adapter = createAdapter(extension);
    auto& file_adapter = static_cast<FileAdapter&>(*data_adapter);
    file_adapter.extendWrite(tables, fileName);
}

std::string 
FileAdapter::findExtension(const std::string& filename) {
    std::size_t found = filename.find_last_of('.');

    OPENSIM_THROW_IF(found == std::string::npos,
                     FileExtensionNotFound,
                     filename);

    return filename.substr(found + 1);
}

std::vector<std::string> 
FileAdapter::tokenize(const std::string& str, 
                      const std::string& delims) const {
    using size_type = std::string::size_type;

    std::vector<std::string> tokens{};

    size_type token_start{0}, token_end{0};
    bool is_token{false};
    while(token_end < str.length()) {
        if(delims.find_first_of(str[token_end]) != std::string::npos) {
            if(is_token) {
                tokens.push_back(str.substr(token_start, 
                                            token_end - token_start));
                is_token = false;
            }
        } else {
            if(!is_token) {
                token_start = token_end;
                is_token = true;
            }
        }

        ++token_end;
    }
    if(is_token)
        tokens.push_back(str.substr(token_start, token_end - token_start));

    return tokens;
}

std::vector<std::string>
FileAdapter::getNextLine(std::istream& stream,
                         const std::string& delims) const {
    std::string line{};
    while(std::getline(stream, line)) {
        auto tokens = tokenize(line, delims);
        if(tokens.size() > 0)
            return tokens;
    }
    return {};
}

} // namespace OpenSim
