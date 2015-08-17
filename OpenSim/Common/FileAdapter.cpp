#include "FileAdapter.h"

namespace OpenSim {

std::unique_ptr<FileAdapter> 
FileAdapter::createAdapter(const std::string& identifier) {
    auto data_adapter_ptr = 
        DataAdapter::createAdapter(identifier).release();
    FileAdapter* file_adapter_ptr = 
        dynamic_cast<FileAdapter*>(data_adapter_ptr);
    return std::unique_ptr<FileAdapter>{file_adapter_ptr};
}

void 
FileAdapter::setFilename(const std::string& filename) {
    filename_ = filename;
}

const std::string& 
FileAdapter::getFilename() const {
    return filename_;
}

std::string 
FileAdapter::findExtension(const std::string& filename) {
    std::size_t found = filename.find_last_of('.');
    return found == std::string::npos ? 
        std::string{} : filename.substr(found + 1);
}

std::vector<std::string> 
FileAdapter::tokenize(const std::string& str, 
                      const std::string& delims) {
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
