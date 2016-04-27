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

// These tests are fairly weak, and are mostly about syntax.
// We do not test *all* possible commands here, since some commands would
// invoke a long computation (e.g., CMC). We mostly just test incorrect input.
// Also, we only test the console output and return code of the commands; we
// don't test the actual purpose of the commands (e.g., if a command was
// supposed to write a file, we don't check that the file was written).

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
CommandOutput system_output(const std::string& command) {
    // http://stackoverflow.com/questions/478898/
    // how-to-execute-a-command-and-get-output-of-command-within-c-using-posix
    // The 2>& 1 redirects stderr to stdout.
    std::string result = "";
    FILE* pipe = popen((command + " 2>& 1").c_str(), "r");
    try {
        if (!pipe) return CommandOutput(-1, "Could not run command.");
        char buffer[128];
        while (!feof(pipe)) {
            if (fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw std::runtime_error("Exception thrown while running command.");
    }
    int returncode = pclose(pipe) / 256;
    if (returncode != 0) returncode = EXIT_FAILURE; // TODO hack
    return CommandOutput(returncode, result);
}

// Test that the command produces exactly the expected output.
void testCommand(const std::string& arguments,
                 int expectedReturnCode,
                 const std::string& expectedOutput) {
    CommandOutput out = system_output(COMMAND + " " + arguments);

    const bool outputIsEqual = (out.output == expectedOutput);
    if (!outputIsEqual) {
        std::string msg = "When testing arguments '" + arguments +
            "' got the following output:\n" + out.output +
            "\nExpected:\n" + expectedOutput;
        throw std::runtime_error(msg);
    }

    const bool returnCodeIsCorrect = (out.returncode == expectedReturnCode);
    if (!returnCodeIsCorrect) {
        std::string msg = "When testing arguments '" + arguments +
            "' got return code '" + std::to_string(out.returncode) +
            "' but expected '" + std::to_string(expectedReturnCode) + "'.";
        throw std::runtime_error(msg);
    }
}

// Test that the command's output matches the given regular expression.
void testCommand(const std::string& arguments,
                 int expectedReturnCode,
                 const std::regex& expectedOutput) {
    CommandOutput out = system_output(COMMAND + " " + arguments);

    std::smatch sm;
    if (!std::regex_match(out.output, sm, expectedOutput)) {
        std::string msg = "When testing arguments '" + arguments +
            "' got the following unexpected output:\n" + out.output;
        throw std::runtime_error(msg);
    }

    const bool returnCodeIsCorrect = (out.returncode == expectedReturnCode);
    if (!returnCodeIsCorrect) {
        std::string msg = "When testing arguments '" + arguments +
            "' got return code '" + std::to_string(out.returncode) +
            "' but expected '" + std::to_string(expectedReturnCode) + "'.";
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
        testCommand("", EXIT_SUCCESS, output);
        testCommand("-h", EXIT_SUCCESS, output);
        testCommand("-help", EXIT_SUCCESS, output);
    }

    // Version.
    // ========
    {
        std::regex output("(OpenSim version )(.*)(, build date )(.*)\n");
        testCommand("-V", EXIT_SUCCESS, output);
        testCommand("--version", EXIT_SUCCESS, output);
    }

    // Library option.
    // ===============
    // Syntax errors.
    testCommand("-L", EXIT_FAILURE,
            std::regex("(-L requires an argument)((.|\n|\r)*)"));
    testCommand("--library", EXIT_FAILURE, 
            std::regex("(--library requires an argument)((.|\n|\r)*)"));
    // Must specify a command; can't only list a library to load.
    {
        std::regex output(
            "(Arguments did not match expected patterns)((.|\n|\r)*)");
        // All of these are otherwise valid options for specify libraries to
        // load.
        testCommand("-L x", EXIT_FAILURE, output);
        testCommand("--library x", EXIT_FAILURE, output);
        testCommand("-L=x", EXIT_FAILURE, output);
        testCommand("--library=y", EXIT_FAILURE, output);
        testCommand("-L x --library y -L z", EXIT_FAILURE, output);
        testCommand("-L=x --library=y -L=z", EXIT_FAILURE, output);
    }

    // Unrecognized command.
    // =====================
    testCommand("bleepbloop", EXIT_FAILURE, 
            "'bleepbloop' is not an opensim command. See 'opensim --help'.\n");
}

// TODO allow LoadOpenSimLibrary to take a filename extension.
void testLoadPluginLibraries(const std::string& subcommand) {

    const auto cmd = subcommand + " -h";

    // Nonexistant file.
    // =================
    {
        std::regex output("((.|\n|\r)*)(Failed to load library x)\n");
        // These are all valid ways of specifying libraries.
        testCommand("-L x " + cmd, EXIT_FAILURE, output);
        testCommand("-Lx " + cmd, EXIT_FAILURE, output);
        testCommand("--library x " + cmd, EXIT_FAILURE, output);
        testCommand("--library=x " + cmd, EXIT_FAILURE, output);
        testCommand("-L x --library y " + cmd, EXIT_FAILURE, output);
        testCommand("-Lx --library=y -L z " + cmd, EXIT_FAILURE, output);
    }

    // Load an actual library.
    // =======================
    const std::string lib = MAKE_STRING(OSIM_ACTUATORS_LIB_PATH);
    {
        std::regex output("(Loaded library " + lib + ")((.|\n|\r)*)\n");
        testCommand("-L " + lib + " " + cmd, EXIT_SUCCESS, output);
        testCommand("-L" + lib + " " + cmd, EXIT_SUCCESS, output);
        testCommand("--library " + lib + " " + cmd, EXIT_SUCCESS, output);
        testCommand("--library=" + lib + " " + cmd, EXIT_SUCCESS, output);
    }
    // Load multiple libraries.
    // ========================
    {
        // Well, in this case, we just load the same library multiple times.
        testCommand("-L " + lib + " --library " + lib + " " + cmd,
                EXIT_SUCCESS,
                std::regex("(Loaded library " + lib + ")\n"
                           "(Loaded library " + lib + ")((.|\n|\r)*)\n"));
        testCommand("-L" + lib +
                    " --library=" + lib +
                    " -L " + lib + " " + cmd, EXIT_SUCCESS,
                std::regex("(Loaded library " + lib + ")\n"
                           "(Loaded library " + lib + ")\n"
                           "(Loaded library " + lib + ")((.|\n|\r)*)\n"));
    }
}

void testPrintXML() {
    // Help.
    // =====
    {
        auto output = std::regex("(Print a template XML file )((.|\n|\r)*)");
        testCommand("print-xml -h", EXIT_SUCCESS, output);
        testCommand("print-xml -help", EXIT_SUCCESS, output);
    }

    // Error messages.
    // ===============
    testCommand("print-xml", EXIT_FAILURE,
            std::regex("(Arguments did not match expected patterns)"
                       "((.|\n|\r)*)"));
    testCommand("print-xml x y z", EXIT_FAILURE,
            std::regex("(Unexpected argument: print-xml, x, y, z)"
                       "((.|\n|\r)*)"));
    testCommand("print-xml bleepbloop", EXIT_FAILURE,
            "There is no tool or class named 'bleepbloop'.\n"
            "Did you intend to load a plugin (with --library)?\n");
    testCommand("print-xml bleepbloop y", EXIT_FAILURE,
            "There is no tool or class named 'bleepbloop'.\n"
            "Did you intend to load a plugin (with --library)?\n");

    // Successful input.
    // =================
    testCommand("print-xml cmc", EXIT_SUCCESS,
            "Printing 'default_CMCTool.xml'.\n");
    testCommand("print-xml Millard2012EquilibriumMuscle", EXIT_SUCCESS,
            "Printing 'default_Millard2012EquilibriumMuscle.xml'.\n");
    testCommand("print-xml cmc default_cmc_setup.xml", EXIT_SUCCESS,
            "Printing 'default_cmc_setup.xml'.\n");

    // Library option.
    // ===============
    testLoadPluginLibraries("print-xml");
}

void testInfo() {
    // Help.
    // =====
    {
        auto output = std::regex("(Show description )((.|\n|\r)*)");
        testCommand("info -h", EXIT_SUCCESS, output);
        testCommand("info -help", EXIT_SUCCESS, output);
    }

    // Error messages.
    // ===============
    testCommand("info x", EXIT_FAILURE,
            "No registered class with name 'x'. "
            "Did you intend to load a plugin?\n");
    testCommand("info x y", EXIT_FAILURE,
            "No registered class with name 'x'. "
            "Did you intend to load a plugin?\n");
    testCommand("info Body y", EXIT_FAILURE,
            "No property with name 'y' found in class 'Body'.\n");

    // Successful input.
    // =================
    testCommand("info", EXIT_SUCCESS,
            std::regex("(REGISTERED CLASSES )((.|\n|\r)*)"));
    testCommand("info PathSpring", EXIT_SUCCESS,
            std::regex("\n(PROPERTIES FOR PathSpring)((.|\n|\r)*)"));
    testCommand("info Body mass", EXIT_SUCCESS,
            "\nBody.mass\nThe mass of the body (kg)\n");

    // Library option.
    // ===============
    testLoadPluginLibraries("info");
}

int main() {
    SimTK_START_TEST("testCommandLineInterface");
        SimTK_SUBTEST(testNoCommand);
        SimTK_SUBTEST(testPrintXML);
        SimTK_SUBTEST(testInfo);
    SimTK_END_TEST();
}
