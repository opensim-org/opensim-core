#include "FileAdapter.h"

namespace OpenSim {

FileAdapter::OutputTables
FileAdapter::readFile(const std::string& fileName) {
    auto extension = findExtension(fileName);
    auto& file_adapter = static_cast<FileAdapter&>(*createAdapter(extension));
    return file_adapter.extendRead(fileName);
}

void 
FileAdapter::writeFile(const InputTables& tables, 
                       const std::string& fileName) {
    auto extension = findExtension(fileName);
    auto& file_adapter = static_cast<FileAdapter&>(*createAdapter(extension));
    file_adapter.extendWrite(tables, fileName);
}

std::string 
FileAdapter::findExtension(const std::string& filename) {
    std::size_t found = filename.find_last_of('.');
    return found == std::string::npos ? 
        std::string{} : filename.substr(found + 1);
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

    return tokens;
}

} // namespace OpenSim
