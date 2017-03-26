/* -------------------------------------------------------------------------- *
 *                   OpenSim:  testCommandLineInterface.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2016-2017 Stanford University and the Authors                *
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

#include <SimTKcommon/Testing.h>
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


// Helper code.
// ============

// The ?: says not to capture the group; should be slightly faster.
// [\s\S]* escapes any amount of whitespace and non-whitespace; the 
// double \\ is to escape the slash.
const std::string RE_ANY = "(?:[\\s\\S]*)";

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
inline int pclose(FILE* file) { 
    return _pclose(file); 
}
#endif

// Execute a system command and also grab its console output.
CommandOutput system_output(std::string command) {
    // http://stackoverflow.com/questions/478898/
    // how-to-execute-a-command-and-get-output-of-command-within-c-using-posix
    // The 2>& 1 redirects stderr to stdout.
    std::string result = "";
    #ifdef _WIN32
        // To achieve proper quoting with cmd.exe, we must surround the
        // entire command with quotes ("). See "cmd.exe /?" for more info.
        command = "\"" + command + "\"";
    #endif
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
    int returncode = pclose(pipe);
    // I was unable to actually get the correct return code on either OSX
    // or Windows, so we will only do a binary check for failure/success.
    // This is fine, considering that we only use two different return
    // codes in the command line interface. -chrisdembia
    if (returncode != 0) returncode = EXIT_FAILURE;
    return CommandOutput(returncode, result);
}

class StartsWith {
public:
    StartsWith(std::string prefix) : prefix(prefix) {}
    bool check(const std::string& str) const {
        return std::equal(prefix.begin(), prefix.end(), str.begin());
    }
    const std::string prefix;
};

// Checks that the command produces exactly the expected output.
void checkCommandOutput(const std::string& arguments,
        const std::string& output,
        const std::string& expectedOutput) {
    const bool outputIsEqual = (output == expectedOutput);
    if (!outputIsEqual) {
        std::string msg = "When testing arguments '" + arguments +
            "' got the following output:\n" + output +
            "\nExpected:\n" + expectedOutput;
        throw std::runtime_error(msg);
    }
}

// Checks that the command output starts with a given string.
// Created this because profiling indicated regexes were making the test slow;
// ended up not being true.
void checkCommandOutput(const std::string& arguments,
        const std::string& output,
        const StartsWith& expectedOutput) {
    if (!expectedOutput.check(output)) {
        std::string msg = "When testing arguments '" + arguments +
            "' got the following output:\n" + output +
            "\nExpected it to start with:\n" + expectedOutput.prefix;
        throw std::runtime_error(msg);
    }
}

// Checks that the command's output matches the given regular expression.
void checkCommandOutput(const std::string& arguments,
        const std::string& output,
        const std::regex& expectedOutput) {
    if (!std::regex_match(output, expectedOutput)) {
        std::string msg = "When testing arguments '" + arguments +
            "' got the following unexpected output:\n" + output;
        throw std::runtime_error(msg);
    }
}

template <typename T>
void testCommand(const std::string& arguments,
                 int expectedReturnCode,
                 const T& expectedOutput) {
    CommandOutput out = system_output(COMMAND + " " + arguments);

    checkCommandOutput(arguments, out.output, expectedOutput);

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
        std::regex output("OpenSim: musculoskeletal" + RE_ANY +
                          "Pass -h or --help" + RE_ANY);
        testCommand("", EXIT_SUCCESS, output);
        testCommand("-h", EXIT_SUCCESS, output);
        testCommand("-help", EXIT_SUCCESS, output);
    }

    // Version.
    // ========
    {
        std::regex output("OpenSim version (?:.*), build date (?:.*)\n");
        testCommand("-V", EXIT_SUCCESS, output);
        testCommand("--version", EXIT_SUCCESS, output);
    }

    // Library option.
    // ===============
    // Syntax errors.
    testCommand("-L", EXIT_FAILURE, StartsWith("-L requires an argument"));
    testCommand("--library", EXIT_FAILURE, 
            StartsWith("--library requires an argument"));
    // Must specify a command; can't only list a library to load.
    {
        StartsWith output("Arguments did not match expected patterns");
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
            "'bleepbloop' is not an opensim-cmd command. "
            "See 'opensim-cmd --help'.\n");
}

// http://stackoverflow.com/questions/5343190/how-do-i-replace-all-instances-of-a-string-with-another-string
std::string replaceString(std::string subject, const std::string& search,
    const std::string& replace) {
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != std::string::npos) {
        subject.replace(pos, search.length(), replace);
        pos += replace.length();
    }
    return subject;
}

void testLoadPluginLibraries(const std::string& subcommand) {

    const auto cmd = subcommand + " -h";

    // Nonexistent file.
    // =================
    {
        std::regex output(RE_ANY + "(Failed to load library x)\n");
        // These are all valid ways of specifying libraries.
        testCommand("-L x " + cmd, EXIT_FAILURE, output);
        testCommand("-Lx " + cmd, EXIT_FAILURE, output);
        testCommand("--library x " + cmd, EXIT_FAILURE, output);
        testCommand("--library=x " + cmd, EXIT_FAILURE, output);
        testCommand("-L x --library y " + cmd, EXIT_FAILURE, output);
        testCommand("-Lx --library=y -L z " + cmd, EXIT_FAILURE, output);
    }

    // Load an actual library, including the file extension.
    // =====================================================
    // OSIM_ACTUATORS_LIB_PATH is a preprocessor definition that is defined
    // when compiling this executable.
    std::string lib = MAKE_STRING(OSIM_ACTUATORS_LIB_PATH);

    // Get rid of the quotes surrounding `lib`.
    std::string expectLib = lib.substr(1, lib.size() - 2);
    #ifdef _WIN32
        // When the library name gets printed back to us, the 
        // forward slashes are converted to backslashes. We have to
        // escape backslash for the C++ parser, so '\\' is actually '\'.
        expectLib = replaceString(expectLib, "/", "\\");
    #endif
    {
        StartsWith output("Loaded library " + expectLib);
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
                StartsWith("Loaded library " + expectLib + "\n"
                           "Loaded library " + expectLib + "\n"));
        testCommand("-L" + lib +
                    " --library=" + lib +
                    " -L " + lib + " " + cmd, EXIT_SUCCESS,
                StartsWith("Loaded library " + expectLib + "\n"
                           "Loaded library " + expectLib + "\n"
                           "Loaded library " + expectLib + "\n"));
    }
}

void testRunTool() {
    // Help.
    // =====
    {
        StartsWith output("Run a tool ");
        testCommand("run-tool -h", EXIT_SUCCESS, output);
        testCommand("run-tool -help", EXIT_SUCCESS, output);
    }

    // Error messages.
    // ===============
    testCommand("run-tool", EXIT_FAILURE,
            StartsWith("Arguments did not match expected patterns"));
    testCommand("run-tool putes.xml", EXIT_FAILURE,
            StartsWith("SimTK Exception thrown at Xml.cpp"));
    // We use print-xml to create a setup file that we can try to run.
    // (We are not really trying to test print-xml right now.)
    testCommand("print-xml cmc testruntool_cmc_setup.xml", EXIT_SUCCESS,
            "Printing 'testruntool_cmc_setup.xml'.\n");
    // This fails because this setup file doesn't have much in it.
    testCommand("run-tool testruntool_cmc_setup.xml", EXIT_FAILURE,
            std::regex("(Preparing to run CMCTool.)" + RE_ANY +
                       "(Running tool default.)" + RE_ANY +
                       "(ERROR- A model has not been set.)" + RE_ANY));
    // Similar to the previous two commands, except for scaling
    // (since ScaleTool goes through a different branch of the code).
    testCommand("print-xml scale testruntool_scale_setup.xml", EXIT_SUCCESS,
            "Printing 'testruntool_scale_setup.xml'.\n");
    // This fails because this setup file doesn't have much in it.
    testCommand("run-tool testruntool_scale_setup.xml", EXIT_FAILURE,
            std::regex("(Preparing to run ScaleTool.)" + RE_ANY +
                       "(Processing subject default)" + RE_ANY));
    // Now we'll try loading a valid OpenSim XML file that is *not* a Tool
    // setup file, and we get a helpful error.
    // (We are not really trying to test print-xml right now.)
    testCommand("print-xml Model testruntool_Model.xml", EXIT_SUCCESS,
            "Printing 'testruntool_Model.xml'.\n");
    testCommand("run-tool testruntool_Model.xml", EXIT_FAILURE,
            "The provided file 'testruntool_Model.xml' does not define "
            "an OpenSim Tool. Did you intend to load a plugin?\n");

    // Library option.
    // ===============
    testLoadPluginLibraries("run-tool");
}

void testPrintXML() {
    // Help.
    // =====
    {
        StartsWith output("Print a template XML file ");
        testCommand("print-xml -h", EXIT_SUCCESS, output);
        testCommand("print-xml -help", EXIT_SUCCESS, output);
    }

    // Error messages.
    // ===============
    testCommand("print-xml", EXIT_FAILURE,
            StartsWith("Arguments did not match expected patterns"));
    testCommand("print-xml x y z", EXIT_FAILURE,
            StartsWith("Unexpected argument: print-xml, x, y, z"));
    testCommand("print-xml bleepbloop", EXIT_FAILURE,
        "There is no tool or registered concrete class named 'bleepbloop'.\n"
        "Did you intend to load a plugin (with --library)?\n");
    testCommand("print-xml bleepbloop y", EXIT_FAILURE,
        "There is no tool or registered concrete class named 'bleepbloop'.\n"
        "Did you intend to load a plugin (with --library)?\n");

    // Successful input.
    // =================
    testCommand("print-xml cmc", EXIT_SUCCESS,
            "Printing 'default_Setup_CMCTool.xml'.\n");
    testCommand("print-xml Millard2012EquilibriumMuscle", EXIT_SUCCESS,
            "Printing 'default_Millard2012EquilibriumMuscle.xml'.\n");
    testCommand("print-xml cmc default_cmc_setup.xml", EXIT_SUCCESS,
            "Printing 'default_cmc_setup.xml'.\n");

    // Tool names are case-insensitive.
    // ================================
    testCommand("print-xml CmC", EXIT_SUCCESS,
            "Printing 'default_Setup_CMCTool.xml'.\n");
    testCommand("print-xml FORwarD", EXIT_SUCCESS,
            "Printing 'default_Setup_ForwardTool.xml'.\n");
    testCommand("print-xml Analyze default_analyze_setup.xml", EXIT_SUCCESS,
            "Printing 'default_analyze_setup.xml'.\n");

    // Library option.
    // ===============
    testLoadPluginLibraries("print-xml");
}

void testInfo() {
    // Help.
    // =====
    {
        StartsWith output("Show description ");
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
    testCommand("info", EXIT_SUCCESS, StartsWith("REGISTERED CLASSES "));
    testCommand("info PathSpring", EXIT_SUCCESS,
            StartsWith("\nPROPERTIES FOR PathSpring"));
    testCommand("info Body mass", EXIT_SUCCESS,
            "\nBody.mass\nThe mass of the body (kg)\n");

    // Library option.
    // ===============
    testLoadPluginLibraries("info");
}

void testUpdateFile() {
    // Help.
    // =====
    {
        StartsWith output("Update an .osim, .xml ");
        testCommand("update-file -h", EXIT_SUCCESS, output);
        testCommand("update-file -help", EXIT_SUCCESS, output);
    }

    // Error messages.
    // ===============

    // Syntax errors.
    testCommand("update-file", EXIT_FAILURE,
            StartsWith("Arguments did not match expected patterns"));
    testCommand("update-file x", EXIT_FAILURE, 
            StartsWith("Arguments did not match expected patterns"));
    testCommand("update-file x.doc", EXIT_FAILURE, 
            StartsWith("Arguments did not match expected patterns"));
    testCommand("update-file x.xml", EXIT_FAILURE, 
            StartsWith("Arguments did not match expected patterns"));
    testCommand("update-file x y", EXIT_FAILURE, 
            "Input file 'x' does not have an extension.\n");
    testCommand("update-file x.doc y", EXIT_FAILURE, 
            "Input file 'x.doc' has an unrecognized extension.\n");

    // File does not exist.
    testCommand("update-file x.xml y", EXIT_FAILURE, 
            std::regex("(?:Loading input file 'x.xml')" + RE_ANY +
                       "(?:Could not make object from file 'x.xml'.\n" + 
                       "Did you intend to load a plugin (with --library)?)" +
                       RE_ANY));
    testCommand("update-file x.osim y", EXIT_FAILURE, 
            std::regex("(?:Loading input file 'x.osim')" + RE_ANY +
                       "(?:Could not make object from file 'x.osim'.\n" +
                       "Did you intend to load a plugin (with --library)?)" +
                       RE_ANY));
    testCommand("update-file x.sto y", EXIT_FAILURE, 
            std::regex("(Loading input file 'x.sto')" + RE_ANY +
                       "(Storage: ERROR- failed to open file x.sto)" + RE_ANY));

    // Successful input.
    // =================
    // We use print-xml to create a file that we can try to update.
    // (We are not really trying to test print-xml right now.)
    testCommand("print-xml Model testupdatefile_Model.osim", EXIT_SUCCESS,
            "Printing 'testupdatefile_Model.osim'.\n");
    testCommand("update-file testupdatefile_Model.osim "
                "testupdatefile_Model_updated.osim", EXIT_SUCCESS,
            "Loading input file 'testupdatefile_Model.osim'.\n"
            "Printing updated file to 'testupdatefile_Model_updated.osim'.\n");

    // Library option.
    // ===============
    testLoadPluginLibraries("update-file");
}

int main() {
    SimTK_START_TEST("testCommandLineInterface");
        SimTK_SUBTEST(testNoCommand);
        SimTK_SUBTEST(testRunTool);
        SimTK_SUBTEST(testPrintXML);
        SimTK_SUBTEST(testInfo);
        SimTK_SUBTEST(testUpdateFile);
    SimTK_END_TEST();
}
