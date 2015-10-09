/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testFilesystemPathProperty.cpp             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/FilesystemPath.h>

using namespace OpenSim;
using namespace SimTK;

class MyObject : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MyObject, Object);
public:
    OpenSim_DECLARE_PROPERTY(number, int, "A number.");
    OpenSim_DECLARE_LIST_PROPERTY(strings, std::string, "ahh");
    OpenSim_DECLARE_PROPERTY(single_path, FilesystemPath, "FilesystemPath to 1 file.");
    OpenSim_DECLARE_PROPERTY(single_path_notempty, FilesystemPath,
            "FilesystemPath to 1 file.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(optional_file, FilesystemPath, "Optional.");
    OpenSim_DECLARE_LIST_PROPERTY(multiple_paths, FilesystemPath,
            "FilesystemPath to multiple files.");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(atmost_3_paths, FilesystemPath, 3, "At most 3 paths.");
    //OpenSim_DECLARE_LIST_PROPERTY_ATLEAST(atleast_2_paths, FilesystemPath, 2, "At least 3 paths.");
    //OpenSim_DECLARE_LIST_PROPERTY_RANGE(two_to_four_paths, FilesystemPath, 2, 4, "Two to 4 paths");
    //OpenSim_DECLARE_LIST_PROPERTY_SIZE(exactly_4_paths, FilesystemPath, 4,
    //        "Exactly 4 paths.");

    // Should not work.
    //OpenSim_DECLARE_UNNAMED_PROPERTY(single_path, FilesystemPath, "FilesystemPath to 1 file.");

    MyObject() {
        constructProperties();
    }

    explicit MyObject(const std::string& fileName)
        : Object(fileName) { 
        constructProperties();
        updateFromXMLDocument();
    }
private:
    void constructProperties() {
        constructProperty_number(0);
        constructProperty_strings();
        constructProperty_single_path(FilesystemPath());
        constructProperty_single_path_notempty(FilesystemPath("tree/branch"));
        constructProperty_optional_file();
        constructProperty_multiple_paths();
        constructProperty_atmost_3_paths();
        /*SimTK::Array<FilesystemPath> v({FilesystemPath(),
                    FilesystemPath(),
                    FilesystemPath(),
                    FilesystemPath()});
        constructProperty_exactly_4_paths(v);*/
    }
};

void testFilesystemPathProperty() {
    MyObject obj;
    obj.set_number(5);
    obj.append_strings("abcd");
    obj.append_strings("half full");
    obj.append_strings("nearness");
    obj.set_single_path(FilesystemPath("x/y/z"));
    obj.set_optional_file(FilesystemPath("helmet/gazebo/frog.xml"));
    obj.append_multiple_paths(FilesystemPath("tendon/muscle/viapoint"));
    obj.append_atmost_3_paths(FilesystemPath("red/green/blue"));
    obj.append_atmost_3_paths(FilesystemPath("yellow/violet"));

    obj.print("pathname_property_MyObject.xml");

    MyObject obj2("pathname_property_MyObject.xml");

    SimTK_TEST(obj == obj2);

    // TODO
    /*
    SimTK_TEST(!obj.getProperty_number.isObjectProperty());
    SimTK_TEST(!obj.getProperty_number.isOneObjectProperty());
    SimTK_TEST(!obj.getProperty_number.isUnnamedObjectProperty());
    */

    // TODO test spaces within pathnames.


    // TODO test spaces before the pathname.
    obj.set_single_path(FilesystemPath("  queen/king"));
    obj.set_optional_file(FilesystemPath("jack/joker  "));

};

int main() {
    SimTK_START_TEST("testFilesystemPathProperty");
        SimTK_SUBTEST(testFilesystemPathProperty);
    SimTK_END_TEST();
}
