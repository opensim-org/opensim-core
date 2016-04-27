/* -------------------------------------------------------------------------- *
 *                   OpenSim:  testCommandLineInterface.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2016 Stanford University and the Authors                     *
 * Author(s): Chris Dembia                                                    *
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

#include <Simbody.h>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <regex>
// We do *not* include OpenSim headers, since we are only interacting with
// OpenSim through its command-line interface. But we do use Simbody's testing
// macros.

// Note: We do not test *all* possible commands here, since some commands would
// invoke a long computation (e.g., CMC). We mostly just test incorrect input.

// OSIM_CLI_PATH is a preprocessor definition that is defined when compiling
// this executable.
#define STR(var) #var
#define MAKE_STRING(a) STR(a)
const std::string COMMAND = MAKE_STRING(OSIM_CLI_PATH);

// For packaging the return code and the console output of a system command.
struct CommandOutput {
    CommandOutput(int returncode, std::string output)
        : returncode(returncode), output(output) {}
    int returncode;
    std::string output;
};

// Cross-platform pipe, popen, pclose.
// http://stackoverflow.com/questions/12402578/crossplatform-lightweight-wrapper-for-pipe-popen
#ifdef _WIN32
inline FILE* popen(const char* command, const char* type) {
    return _popen(command, type);
}
inline void pclose(FILE* file) { 
    _pclose(file); 
}
#endif

// Execute a system command and also grab its console output.
// TODO set status so we can test the expected status.
CommandOutput system_output(const std::string& command) {
    // http://stackoverflow.com/questions/478898/
    // how-to-execute-a-command-and-get-output-of-command-within-c-using-posix
    // The 2>& 1 redirects stderr to stdout.
    std::shared_ptr<FILE> pipe(popen((command + " 2>& 1").c_str(), "r"), pclose);
    if (!pipe) return CommandOutput(-1, "ERROR");
    char buffer[128];
    std::string result = "";
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    return CommandOutput(-1, result);
}

// TODO document these methods.
void testCommand(const std::string& arguments,
                 const std::string& expectedOutput,
                 int expectedStatus = 0) {
    CommandOutput out = system_output(COMMAND + " " + arguments);

    const bool outputIsEqual = (out.output == expectedOutput);
    if (!outputIsEqual) {
        std::string msg = "When testing arguments '" + arguments +
            "' got the following output:\n" + out.output +
            "\nExpected:\n" + expectedOutput;

        // TODO SimTK_TEST_FAILED(msg.c_str());
        throw std::runtime_error(msg);
        // TODO SimTK_TEST_FAILED2("Expected:\n%s\nActual:%s",
        // TODO         expectedOutput.c_str(), out.output.c_str());
        // TODO std::cout << "Output: " << std::endl;
        // TODO std::cout << out.output << std::endl;
    }
}

void testCommand(const std::string& arguments,
                 const std::regex& expectedOutput,
                 int expectedStatus = 0) {
    CommandOutput out = system_output(COMMAND + " " + arguments);

    std::smatch sm;
    if (!std::regex_match(out.output, sm, expectedOutput)) {
        std::string msg = "When testing arguments '" + arguments +
            "' got the following unexpected output:\n" + out.output;
        throw std::runtime_error(msg);
    }
}

void testNoCommand() {
    // Help.
    // =====
    {
        // Match REGISTERED CLASSES and then any amount of text and newlines.
        // The pattern (.|\n|\r)* matches any amount of text and newlines.
        std::regex output("(OpenSim: musculoskeletal)"
                          "((.|\n|\r)*)"
                          "(Pass -h or --help)"
                          "((.|\n|\r)*)");
        testCommand("", output);
        testCommand("-h", output);
        testCommand("-help", output);
    }

    // Version.
    // ========
    {
        std::regex output("(OpenSim version )(.*)(, build date )(.*)\n");
        testCommand("-V", output);
        testCommand("--version", output);
    }

    // Library option.
    // ===============
    // Syntax errors.
    testCommand("-L", std::regex("(-L requires an argument)((.|\n|\r)*)"));
    testCommand("--library",
            std::regex("(--library requires an argument)((.|\n|\r)*)"));
    // Must specify a command; can't only list a library to load.
    {
        std::regex output(
            "(Arguments did not match expected patterns)((.|\n|\r)*)");
        testCommand("-L x", output);
        testCommand("--library y", output);
    }
    // TODO testCommand("-L x -L y --library z", output);
    // TODO testCommand("--library y", output);
    // TODO testCommand("-L=x", "TODO");
    // TODO testCommand("--library=y", "TODO");

    // TODO test actually loading libraries (with each subcommand).

    // Unrecognized command.
    // =====================
    testCommand("bleepbloop",
            "'bleepbloop' is not an opensim command. See 'opensim --help'.\n");
}

void testInfo() {
    // Help.
    // =====
    {
        auto output = std::regex("(Show description )((.|\n|\r)*)");
        testCommand("info -h", output);
        testCommand("info -help", output);
    }

    // Error messages.
    // ===============
    testCommand("info x",
            "No registered class with name 'x'. "
            "Did you intend to load a plugin?\n");
    testCommand("info x y",
            "No registered class with name 'x'. "
            "Did you intend to load a plugin?\n");
    testCommand("info Body y",
            "No property with name 'y' found in class 'Body'.\n");

    // Successful input.
    // =================
    testCommand("info", std::regex("(REGISTERED CLASSES )((.|\n|\r)*)"));
    testCommand("info PathSpring",
            std::regex("\n(PROPERTIES FOR PathSpring)((.|\n|\r)*)"));
    testCommand("info Body mass", "\nBody.mass\nThe mass of the body (kg)\n");

    // TODO passing library option.
    // Library option.
    // ===============
    testCommand("-L x info",
            std::regex("((.|\n|\r)*)(Failed to load library x)\n"));
}

int main() {
    SimTK_START_TEST("testCommandLineInterface");
        SimTK_SUBTEST(testNoCommand);
        SimTK_SUBTEST(testInfo);
    SimTK_END_TEST();
}
