#ifndef OPENSIM_JAM_UTILITIES_H_
#define OPENSIM_JAM_UTILITIES_H_
/* -------------------------------------------------------------------------- *
 *                             JAMUtilities.h                              *
 * -------------------------------------------------------------------------- *
 * Author(s): Colin Smith                                                     *
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


//=============================================================================
// INCLUDES
//=============================================================================
//#include <OpenSim/Simulation/Model/Analysis.h>
#include <string> 
#include <vector>

//=============================================================================
//=============================================================================
//namespace OpenSim {


//=============================================================================
//STRING TOOLS
//=============================================================================
	
/** Split string at delimiter
*/
std::vector<std::string> split_string(std::string s, std::string delimiter);

bool contains_string(std::vector<std::string> s_vector, std::string s);
bool contains_string(std::vector<std::string> s_vector, std::string s, int& index);
int find_nearest(std::vector<double> in_vec, double value);
std::string erase_sub_string(std::string mainStr, const std::string & toErase);


//}; //namespace
#endif // #ifndef OPENSIM_JAM_UTILITIES_H_
