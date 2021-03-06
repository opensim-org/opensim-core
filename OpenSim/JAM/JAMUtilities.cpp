/* -------------------------------------------------------------------------- *
 *                           HelperFunctions.cpp                              *
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
#include "JAMUtilities.h"
#include <algorithm>
#include <vector>

//=============================================================================
// String Tools
//=============================================================================
std::vector<std::string> split_string(std::string s, std::string delimiter)
{
    std::vector<std::string> split_s;
    std::vector<std::string> delim;
    
    for (int i = 0; i < delimiter.size(); i++) {
        delim.push_back(std::string(1,delimiter[i]));
    }
    
    std::string found_d;
    size_t pos = s.length();

    while (1) {
        found_d="none";
        pos = s.length();
        for (auto d : delim) {
            if(s.find(d) < pos) {
                pos = s.find(d);
                found_d = d;
            }
        }

        if (found_d == "none") {
            break;
        }

        split_s.push_back(s.substr(0, pos));

        s.erase(0, pos + found_d.length());
    }
    split_s.push_back(s.substr(0, pos));
    return split_s;
}



bool contains_string(std::vector<std::string> s_vector, std::string s)
{
    int index;
    return contains_string(s_vector, s, index);
}

bool contains_string(std::vector<std::string> s_vector, std::string s, int& index)
{
    bool found;

    auto it = std::find(s_vector.begin(), s_vector.end(), s);

    if (it == s_vector.end()) {
        found = false;
        index = -1;
    }
    else {
        found = true;
        index = (int)std::distance(s_vector.begin(), it);
    }
    return found;
}


/*
 * Erase First Occurrence of given  substring from main string.
 */
std::string erase_sub_string(std::string mainStr, const std::string & toErase)
{
    // Search for the substring in string
    size_t pos = mainStr.find(toErase);
 
    if (pos != std::string::npos)
    {
        // If found then erase it from string
        mainStr.erase(pos, toErase.length());
    }

    return mainStr;
}

int find_nearest(std::vector<double> in_vec, double value)
{
    std::vector<double>::iterator low = std::lower_bound(in_vec.begin(), in_vec.end(), value);
    
    return int(low - in_vec.begin());
}

/*SimTK::Matrix sort_matrix_by_column(SimTK::Matrix& matrix, int col) {
    std::vector<std::vector<double>> sort_matrix;
    sort_matrix.resize(matrix.ncol());
    for (int i = 0; i < matrix.ncol(); ++i) {
        sort_matrix[i].resize(matrix.nrow());
        for (int j = 0; j < matrix.nrow(); ++j) {
            sort_matrix[i][j] = matrix(j, i);
        }
    }

    std::sort(sort_matrix.begin(),
            sort_matrix.end(),
            [col](const std::vector<double>& lhs, const std::vector<double>& rhs) {
            return lhs[col] > rhs[col];
        });
    
    SimTK::Matrix out_mat(matrix.nrow(),matrix.ncol());

    for (int i = 0; i < matrix.ncol(); ++i) {
        for (int j = 0; j < matrix.nrow(); ++j) {
            out_mat(j, i) = sort_matrix[i][j];
        }
    }

    return out_mat;

}*/