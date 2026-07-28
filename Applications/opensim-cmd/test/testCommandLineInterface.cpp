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

#include <catch2/catch_all.hpp>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>
// We do *not* include OpenSim headers, since we are only interacting with
// OpenSim through its command-line interface. We use the Catch2 testing
// framework (which also provides main() via Catch2::Catch2WithMain).

// These tests are fairly weak, and are mostly about syntax.
// We do not test *all* possible commands here, since some commands would
// invoke a long computation (e.g., CMC). We mostly just test incorrect input.
// Aside from the log-file test below, we only check the console output and
// return code of the commands; we don't test the actual purpose of the
// commands (e.g., if a command was supposed to write a file, we don't check
// that the file was written).

using namespace Catch::Matchers;

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

// Run opensim-cmd with the given arguments and check that its console output
// satisfies `outputMatcher` (any Catch2 string matcher, e.g. ContainsSubstring,
// StartsWith, Matches) and that it returns `expectedReturnCode`.
void testCommand(const std::string& arguments, int expectedReturnCode,
                 const MatcherBase<std::string>& outputMatcher) {
    CommandOutput out = system_output(COMMAND + " " + arguments);
    INFO("Arguments: " << arguments);
    INFO("Output:\n" << out.output);
    CHECK_THAT(out.output, outputMatcher);
    CHECK(out.returncode == expectedReturnCode);
}

// Some tests need a file to operate on, and use print-xml to create it. Those
// commands are setup, not the thing under test, so a failure here is fatal:
// carrying on would fail the commands that follow for reasons that have nothing
// to do with what they are checking.
void requirePrintedFile(const std::string& classOrTool,
                        const std::string& filepath) {
    const std::string arguments = "print-xml " + classOrTool + " " + filepath;
    CommandOutput out = system_output(COMMAND + " " + arguments);
    INFO("Arguments: " << arguments);
    INFO("Output:\n" << out.output);
    REQUIRE_THAT(out.output,
            ContainsSubstring("Printing '" + filepath + "'.\n"));
    REQUIRE(out.returncode == EXIT_SUCCESS);
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

// Removes the given files on destruction so that a test cleans up after itself
// even when an assertion fails. Catch2 does not provide a temporary-file
// fixture, and this test intentionally does not link osimCommon (whose
// FileRemover lives), so we use a small self-contained std::filesystem guard.
class ScopedFileRemover {
public:
    explicit ScopedFileRemover(std::vector<std::string> paths)
            : m_paths(std::move(paths)) {}
    ~ScopedFileRemover() {
        for (const auto& path : m_paths) {
            std::error_code ec;
            std::filesystem::remove(path, ec);
        }
    }
private:
    std::vector<std::string> m_paths;
};

// Exercises the plugin-loading (--library) option, shared by all subcommands.
void testLoadPluginLibraries(const std::string& subcommand) {

    const auto cmd = subcommand + " -h";

    // Nonexistent file.
    // =================
    {
        auto output = Matches(RE_ANY + "(Failed to load library x)\n");
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
        auto output = StartsWith("[info] Loaded library " + expectLib);
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
                StartsWith("[info] Loaded library " + expectLib + "\n"
                           "[info] Loaded library " + expectLib + "\n"));
        testCommand("-L" + lib +
                    " --library=" + lib +
                    " -L " + lib + " " + cmd, EXIT_SUCCESS,
                StartsWith("[info] Loaded library " + expectLib + "\n"
                           "[info] Loaded library " + expectLib + "\n"
                           "[info] Loaded library " + expectLib + "\n"));
    }
}

TEST_CASE("opensim-cmd with no (or a bad) command", "[opensim-cmd]") {
    SECTION("Help") {
        auto output = Matches("OpenSim: musculoskeletal" + RE_ANY +
                              "Pass -h or --help" + RE_ANY);
        testCommand("", EXIT_SUCCESS, output);
        testCommand("-h", EXIT_SUCCESS, output);
        testCommand("--help", EXIT_SUCCESS, output);
    }

    SECTION("Version") {
        auto output = Matches("OpenSim version (?:.*), build date (?:.*)\n");
        testCommand("-V", EXIT_SUCCESS, output);
        testCommand("--version", EXIT_SUCCESS, output);
    }

    SECTION("Options that require an argument") {
        testCommand("-L", EXIT_FAILURE,
                ContainsSubstring("-L requires an argument"));
        testCommand("--library", EXIT_FAILURE,
                ContainsSubstring("--library requires an argument"));
        testCommand("--log-file", EXIT_FAILURE,
                ContainsSubstring("--log-file requires an argument"));
    }

    SECTION("Options alone are not enough; a command is required") {
        auto output =
                ContainsSubstring("Arguments did not match expected patterns");
        // All of these are otherwise valid ways to specify libraries to load.
        testCommand("-L x", EXIT_FAILURE, output);
        testCommand("--library x", EXIT_FAILURE, output);
        testCommand("-L=x", EXIT_FAILURE, output);
        testCommand("--library=y", EXIT_FAILURE, output);
        testCommand("-L x --library y -L z", EXIT_FAILURE, output);
        testCommand("-L=x --library=y -L=z", EXIT_FAILURE, output);
        // Likewise for the log-file options. Parsing fails before opensim-cmd
        // gets as far as opening a log file, so no file is created here.
        testCommand("--log-file x", EXIT_FAILURE, output);
        testCommand("--log-file=x", EXIT_FAILURE, output);
        testCommand("--no-log-file", EXIT_FAILURE, output);
    }

    SECTION("Unrecognized command") {
        std::string str_bleepbloop(
                "'bleepbloop' is not an opensim-cmd command. "
                "See 'opensim-cmd --help'.\n");
        testCommand("bleepbloop", EXIT_FAILURE,
                ContainsSubstring(str_bleepbloop));
    }
}

TEST_CASE("opensim-cmd run-tool", "[opensim-cmd]") {
    SECTION("Help") {
        auto output = StartsWith("Run a tool ");
        testCommand("run-tool -h", EXIT_SUCCESS, output);
        testCommand("run-tool -help", EXIT_SUCCESS, output);
    }

    SECTION("Syntax errors") {
        testCommand("run-tool", EXIT_FAILURE,
                ContainsSubstring("Arguments did not match expected patterns"));
        testCommand("run-tool putes.xml", EXIT_FAILURE,
                StartsWith("[error] SimTK Exception thrown at"));
    }

    SECTION("A setup file without a model") {
        // We use print-xml to create a setup file that we can try to run.
        // (We are not really trying to test print-xml right now.)
        requirePrintedFile("cmc", "testruntool_cmc_setup.xml");
        // This fails because this setup file doesn't have much in it.
        testCommand("run-tool testruntool_cmc_setup.xml", EXIT_FAILURE,
                Matches(RE_ANY + "(No model file was specified)" + RE_ANY));
    }

    SECTION("A scale setup file without a model") {
        // Similar to the previous section, except for scaling (since ScaleTool
        // goes through a different branch of the code).
        requirePrintedFile("scale", "testruntool_scale_setup.xml");
        testCommand("run-tool testruntool_scale_setup.xml", EXIT_FAILURE,
                Matches(RE_ANY + "(Preparing to run ScaleTool.)" + RE_ANY +
                        "(Processing subject default)" + RE_ANY));
    }

    SECTION("A valid OpenSim XML file that is not a Tool setup file") {
        // We get a helpful error rather than something cryptic.
        requirePrintedFile("Model", "testruntool_Model.xml");
        testCommand("run-tool testruntool_Model.xml", EXIT_FAILURE,
                ContainsSubstring("The provided file 'testruntool_Model.xml' "
                                  "does not define an OpenSim Tool. "
                                  "Did you intend to load a plugin?\n"));
    }

    SECTION("Library option") {
        testLoadPluginLibraries("run-tool");
    }
}

TEST_CASE("opensim-cmd print-xml", "[opensim-cmd]") {
    SECTION("Help") {
        auto output = StartsWith("Print a template XML file ");
        testCommand("print-xml -h", EXIT_SUCCESS, output);
        testCommand("print-xml -help", EXIT_SUCCESS, output);
    }

    SECTION("Error messages") {
        testCommand("print-xml", EXIT_FAILURE,
                ContainsSubstring("Arguments did not match expected patterns"));
        testCommand("print-xml x y z", EXIT_FAILURE,
                ContainsSubstring("Unexpected argument: print-xml, x, y, z"));
        auto noSuchClass =
                ContainsSubstring("There is no tool or registered concrete "
                                  "class named 'bleepbloop'.\nDid you intend "
                                  "to load a plugin (with --library)?\n");
        testCommand("print-xml bleepbloop", EXIT_FAILURE, noSuchClass);
        testCommand("print-xml bleepbloop y", EXIT_FAILURE, noSuchClass);
    }

    SECTION("Successful input") {
        testCommand("print-xml cmc", EXIT_SUCCESS,
                ContainsSubstring("Printing 'default_Setup_CMCTool.xml'.\n"));
        testCommand("print-xml Millard2012EquilibriumMuscle", EXIT_SUCCESS,
                ContainsSubstring(
                        "Printing 'default_Millard2012EquilibriumMuscle.xml'."
                        "\n"));
        testCommand("print-xml cmc default_cmc_setup.xml", EXIT_SUCCESS,
                ContainsSubstring("Printing 'default_cmc_setup.xml'.\n"));
    }

    SECTION("Tool names are case-insensitive") {
        testCommand("print-xml CmC", EXIT_SUCCESS,
                ContainsSubstring("Printing 'default_Setup_CMCTool.xml'.\n"));
        testCommand("print-xml FORwarD", EXIT_SUCCESS,
                ContainsSubstring(
                        "Printing 'default_Setup_ForwardTool.xml'.\n"));
        testCommand("print-xml Analyze default_analyze_setup.xml", EXIT_SUCCESS,
                ContainsSubstring("Printing 'default_analyze_setup.xml'.\n"));
    }

    SECTION("Library option") {
        testLoadPluginLibraries("print-xml");
    }
}

TEST_CASE("opensim-cmd info", "[opensim-cmd]") {
    SECTION("Help") {
        auto output = StartsWith("Show description ");
        testCommand("info -h", EXIT_SUCCESS, output);
        testCommand("info -help", EXIT_SUCCESS, output);
    }

    SECTION("Error messages") {
        auto noSuchClass =
                ContainsSubstring("No registered class with name 'x'. "
                                  "Did you intend to load a plugin?\n");
        testCommand("info x", EXIT_FAILURE, noSuchClass);
        testCommand("info x y", EXIT_FAILURE, noSuchClass);
        testCommand("info Body y", EXIT_FAILURE,
                ContainsSubstring("No property with name 'y' found in "
                                  "class 'Body'.\n"));
    }

    SECTION("Successful input") {
        testCommand("info", EXIT_SUCCESS, StartsWith("REGISTERED CLASSES "));
        testCommand("info PathSpring", EXIT_SUCCESS,
                StartsWith("\nPROPERTIES FOR PathSpring"));
        testCommand("info Body mass", EXIT_SUCCESS,
                ContainsSubstring("\nBody.mass\nThe mass of the body (kg)\n"));
    }

    SECTION("Library option") {
        testLoadPluginLibraries("info");
    }
}

TEST_CASE("opensim-cmd update-file", "[opensim-cmd]") {
    SECTION("Help") {
        auto output = StartsWith("Update an .osim, .xml ");
        testCommand("update-file -h", EXIT_SUCCESS, output);
        testCommand("update-file -help", EXIT_SUCCESS, output);
    }

    SECTION("Syntax errors") {
        auto noMatch =
                ContainsSubstring("Arguments did not match expected patterns");
        testCommand("update-file", EXIT_FAILURE, noMatch);
        testCommand("update-file x", EXIT_FAILURE, noMatch);
        testCommand("update-file x.doc", EXIT_FAILURE, noMatch);
        testCommand("update-file x.xml", EXIT_FAILURE, noMatch);
        testCommand("update-file x y", EXIT_FAILURE,
                ContainsSubstring(
                        "Input file 'x' does not have an extension.\n"));
        testCommand("update-file x.doc y", EXIT_FAILURE,
                ContainsSubstring("Input file 'x.doc' has an unrecognized "
                                  "extension.\n"));
    }

    SECTION("File does not exist") {
        testCommand("update-file x.xml y", EXIT_FAILURE,
                Matches(RE_ANY + "(?:Loading input file 'x.xml')" + RE_ANY +
                        "(?:Could not make object from file 'x.xml'.\n" +
                        "Did you intend to load a plugin (with --library)?)" +
                        RE_ANY));
        testCommand("update-file x.osim y", EXIT_FAILURE,
                Matches(RE_ANY + "(?:Loading input file 'x.osim')" + RE_ANY +
                        "(?:Could not make object from file 'x.osim'.\n" +
                        "Did you intend to load a plugin (with --library)?)" +
                        RE_ANY));
        testCommand("update-file x.sto y", EXIT_FAILURE,
                Matches(RE_ANY + "(Loading input file 'x.sto')" + RE_ANY +
                        "(Storage: Failed to open file 'x.sto')" + RE_ANY));
    }

    SECTION("Successful input") {
        // We use print-xml to create a file that we can try to update.
        // (We are not really trying to test print-xml right now.)
        requirePrintedFile("Model", "testupdatefile_Model.osim");
        testCommand("update-file testupdatefile_Model.osim "
                    "testupdatefile_Model_updated.osim", EXIT_SUCCESS,
                    Matches(RE_ANY + "(Loading input file "
                            "'testupdatefile_Model.osim'.\n)" +
                            RE_ANY + "(Printing updated file to "
                            "'testupdatefile_Model_updated.osim'.\n)" +
                            RE_ANY));
    }

    SECTION("Library option") {
        testLoadPluginLibraries("update-file");
    }
}

TEST_CASE("opensim-cmd log file options", "[opensim-cmd]") {
    // The osimCommon library never creates a log file on its own; opensim-cmd
    // decides the file-logging policy at runtime. We use print-xml as a simple
    // command that succeeds and emits log output. opensim-cmd's addFileSink()
    // creates (opens) the log file immediately, so checking that the file
    // exists is enough.
    const std::string cmd = "print-xml cmc testlogfile_cmc_setup.xml";
    const auto printed =
            ContainsSubstring("Printing 'testlogfile_cmc_setup.xml'.\n");

    // Remove the files these commands may create when this test case goes out
    // of scope (i.e., after every SECTION), so the cases are self-cleaning and
    // independent even if an assertion fails. The last entry is the XML file
    // that `cmd` prints; the rest are the log files under test.
    ScopedFileRemover cleanup({"opensim.log", "testlogfile_custom.log",
                               "testlogfile_spaced.log", "overridden.log",
                               "testlogfile_cmc_setup.xml"});
    // Clear any 'opensim.log' left behind by the other test cases so the
    // negative checks below are meaningful.
    std::error_code ec;
    std::filesystem::remove("opensim.log", ec);

    SECTION("default writes opensim.log in the current directory") {
        testCommand(cmd, EXIT_SUCCESS, printed);
        CHECK(std::filesystem::exists("opensim.log"));
    }

    SECTION("--no-log-file writes no log file") {
        // This also demonstrates that the library does not auto-create a file.
        testCommand("--no-log-file " + cmd, EXIT_SUCCESS, printed);
        CHECK_FALSE(std::filesystem::exists("opensim.log"));
    }

    SECTION("--log-file writes to the given path instead of opensim.log") {
        testCommand("--log-file=testlogfile_custom.log " + cmd, EXIT_SUCCESS,
                printed);
        CHECK(std::filesystem::exists("testlogfile_custom.log"));
        CHECK_FALSE(std::filesystem::exists("opensim.log"));
    }

    SECTION("--log-file also accepts a space-separated value") {
        testCommand("--log-file testlogfile_spaced.log " + cmd, EXIT_SUCCESS,
                printed);
        CHECK(std::filesystem::exists("testlogfile_spaced.log"));
        CHECK_FALSE(std::filesystem::exists("opensim.log"));
    }

    SECTION("--no-log-file overrides --log-file") {
        // Even when a log-file location is given, --no-log-file wins and no
        // file is created (neither the given path nor the default).
        testCommand("--no-log-file --log-file=overridden.log " + cmd,
                EXIT_SUCCESS, printed);
        CHECK_FALSE(std::filesystem::exists("overridden.log"));
        CHECK_FALSE(std::filesystem::exists("opensim.log"));
    }

    SECTION("--log-file that cannot be opened is an error") {
        // Logger::addFileSink() merely warns when it cannot open the file.
        // That is acceptable for the implicit default, but a log file the user
        // asked for by name must not fail silently.
        const std::string badPath = "testlogfile_no_such_dir/x.log";
        REQUIRE_FALSE(std::filesystem::exists("testlogfile_no_such_dir"));
        testCommand("--log-file=" + badPath + " " + cmd, EXIT_FAILURE,
                ContainsSubstring("Could not open log file '" + badPath +
                                  "' for writing."));
        // The command exited before doing any work, so it printed nothing.
        CHECK_FALSE(std::filesystem::exists("testlogfile_cmc_setup.xml"));
        CHECK_FALSE(std::filesystem::exists("opensim.log"));
    }
}
