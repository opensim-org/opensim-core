#include "FileAdapter.h"
#include <OpenSim/Common/IO.h>
#include "STOFileAdapter.h"

namespace OpenSim {

std::shared_ptr<DataAdapter>
createSTOFileAdapterForReading(const std::string&);

std::shared_ptr<DataAdapter>
createSTOFileAdapterForWriting(const DataAdapter::InputTables&);

void 
FileAdapter::writeFile(const InputTables& tables, 
                       const std::string& fileName) {
    auto extension = findExtension(fileName);
    std::shared_ptr<DataAdapter> dataAdapter{};
    if(extension == "sto")
        dataAdapter = createSTOFileAdapterForWriting(tables);
    else
        dataAdapter = createAdapter(extension);
    auto& fileAdapter = static_cast<FileAdapter&>(*dataAdapter);
    fileAdapter.extendWrite(tables, fileName);
}

std::string 
FileAdapter::findExtension(const std::string& filename) {
    std::size_t found = filename.find_last_of('.');

    OPENSIM_THROW_IF(found == std::string::npos,
                     FileExtensionNotFound,
                     filename);

    auto ext = filename.substr(found + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext;
}

std::vector<std::string> 
FileAdapter::tokenize(const std::string& str, 
                      const std::string& delims) {
    using size_type = std::string::size_type;

    std::vector<std::string> tokens{};
    std::string token;

    size_type token_start{0}, token_end{ std::string::npos };
    while((token_end = str.find_first_of(delims, token_start)) 
                    != std::string::npos) {
        token = str.substr(token_start, token_end - token_start);
        OpenSim::IO::TrimWhitespace(token);
        tokens.push_back(token);
        token_start = token_end;
        ++token_start;
    }
    // reached std::string::npos, now set token_end to be the end of the string
    token_end = str.size();
    //capture from last delimiter to the end of string that is not empty
    if (token_end > token_start) {
        token = str.substr(token_start, token_end - token_start);
        OpenSim::IO::TrimWhitespace(token);
        tokens.push_back(token);
    }
    return tokens;
}

std::vector<std::string>
FileAdapter::getNextLine(std::istream& stream,
                         const std::string& delims) {
    std::string line{};
    if(std::getline(stream, line)) {
        // Get rid of the extra \r if parsing a file with CRLF line endings.
        if (!line.empty() && line.back() == '\r') 
            line.pop_back();

        auto tokens = tokenize(line, delims);
        if(tokens.size() > 0)
            return tokens;
    }
    return {};
}

std::shared_ptr<DataAdapter>
FileAdapter::createAdapterFromExtension(const std::string& fileName) {
    auto extension = FileAdapter::findExtension(fileName);
    std::shared_ptr<DataAdapter> dataAdapter{};
    if (extension == "sto")
        dataAdapter = createSTOFileAdapterForReading(fileName);
    else
        dataAdapter = createAdapter(extension);
    return dataAdapter;
}
} // namespace OpenSim
