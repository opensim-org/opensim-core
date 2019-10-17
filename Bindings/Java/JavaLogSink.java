/* -------------------------------------------------------------------------- *
 *                         OpenSim:  JavaLogSink.java                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2019 Stanford University and the Authors                *
 * Author(s): Christopher Dembia                                              *
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
package org.opensim.modeling;

/** Use this class to log OpenSim's messages in Java. This is particularly
useful for Windows Matlab users, for whom OpenSim's messages often do not
appear in Matlab's Command Window. If you have this problem, run the following
code in Matlab:

import org.opensim.modeling.*;
Logger.addSink(JavaLogSink());
*/
public class JavaLogSink extends LogSink {
  protected void sinkImpl(final String msg) {
    System.out.println(msg);
  }
}
