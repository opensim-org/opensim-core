/* Copyright (c)  2008 Stanford University and Ayman Habib.
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Define the standard SimTK compliant "version" and "about" routines.
 */


#include "osimSimmFileWriterDLL.h"

#include <string>
#include <cstring>
#include <cctype>


#define STR(var) #var
#define MAKE_VERSION_STRING(maj,min,build)  STR(maj.min.build)
#define MAKE_COPYRIGHT_STRING(y,a) \
    "Copyright (c) " STR(y) " Stanford University, " STR(a)
#define MAKE_STRING(a) STR(a)

#define GET_VERSION_STRING  \
    MAKE_VERSION_STRING(OPENSIM_SIMMFILEWRITER_MAJOR_VERSION,  \
                        OPENSIM_SIMMFILEWRITER_MINOR_VERSION,  \
                        OPENSIM_SIMMFILEWRITER_BUILD_VERSION)

#define GET_COPYRIGHT_STRING \
    MAKE_COPYRIGHT_STRING(OPENSIM_SIMMFILEWRITER_COPYRIGHT_YEARS, \
                          OPENSIM_SIMMFILEWRITER_AUTHORS)

#define GET_AUTHORS_STRING \
    MAKE_STRING(OPENSIM_SIMMFILEWRITER_AUTHORS)

#define GET_LIBRARY_STRING \
    MAKE_STRING(OPENSIM_SIMMFILEWRITER_LIBRARY_NAME)

#define GET_TYPE_STRING \
    MAKE_STRING(OPENSIM_SIMMFILEWRITER_TYPE)

#ifndef NDEBUG
#define GET_DEBUG_STRING "debug"
#else
#define GET_DEBUG_STRING "release"
#endif


using namespace std;


extern "C" {

    void opensim_version_simulation(int* major, int* minor, int* build) {
        static const char* l = "OPENSIM library="   GET_LIBRARY_STRING;
        static const char* t = "OPENSIM type="      GET_TYPE_STRING;
        static const char* d = "OPENSIM debug="     GET_DEBUG_STRING;
        static const char* v = "OPENSIM version="   GET_VERSION_STRING;
        static const char* c = "OPENSIM copyright=" GET_COPYRIGHT_STRING;

        if (major) *major = OPENSIM_SIMMFILEWRITER_MAJOR_VERSION;
        if (minor) *minor = OPENSIM_SIMMFILEWRITER_MINOR_VERSION;
        if (build) *build = OPENSIM_SIMMFILEWRITER_BUILD_VERSION;

        // Force statics to be present in the binary (Release mode otherwise
        // optimizes them away).
        volatile int i=0;
        if (i) { // never true, but compiler doesn't know ...
            *major = *l + *t + *d + *v + *c;
        }
    }

    void opensim_about_simulation(const char* key, int maxlen, char* value) {
        if (maxlen <= 0 || value==0) return;
        value[0] = '\0'; // in case we don't find a match
        if (key==0) return;

        // downshift the key
        std::string skey(key);
        for (size_t i=0; i<skey.size(); ++i)
            skey[i] = tolower(skey[i]);

        const char* v = 0;
        if      (skey == "version")   v = GET_VERSION_STRING;
        else if (skey == "library")   v = GET_LIBRARY_STRING;
        else if (skey == "type")      v = GET_TYPE_STRING;
        else if (skey == "copyright") v = GET_COPYRIGHT_STRING;
        else if (skey == "authors")   v = GET_AUTHORS_STRING;
        else if (skey == "debug")     v = GET_DEBUG_STRING;

        if (v) {
            strncpy(value,v,maxlen-1);
            value[maxlen-1] = '\0'; // in case we ran out of room
        }
    }

}
