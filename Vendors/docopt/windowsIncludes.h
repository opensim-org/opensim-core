#ifndef DOCOPT_WINDOW_INCLUDE_H_
#define DOCOPT_WINDOW_INCLUDE_H_

/*
 * Shared libraries are messy in Visual Studio. We have to distinguish three
 * cases:
 *   (1) this header is being used to build the docopt shared library
 *       (dllexport)
 *   (2) this header is being used by a *client* of the docopt shared
 *       library (dllimport)
 *   (3) we are building the docopt static library, or the client is
 *       being compiled with the expectation of linking with the
 *       docopt static library (nothing special needed)
 * In the CMake script for building this library, we define one of the symbols
 *     docopt_BUILDING_{SHARED|STATIC}_LIBRARY
 * Client code normally has no special symbol defined, in which case we'll
 * assume it wants to use the shared library. However, if the client defines
 * the symbol docopt_USE_STATIC_LIBRARIES we'll suppress the dllimport so
 * that the client code can be linked with static libraries. Note that
 * the client symbol is not library dependent, while the library symbols
 * affect only the docopt library, meaning that other libraries can
 * be clients of this one. However, we are assuming all-static or all-shared.
 */

#ifdef _MSC_VER
    // We don't want to hear about how sprintf is "unsafe".
    #pragma warning(disable:4996)
    // Keep MS VC++ quiet about lack of dll export of private members.
    #pragma warning(disable:4251)
    #if defined(DOCOPT_BUILDING_SHARED_LIBRARY)
        #define DOCOPT_EXPORT __declspec(dllexport)
    #elif defined(DOCOPT_BUILDING_STATIC_LIBRARY) || defined(DOCOPT_USE_STATIC_LIBRARIES)
        #define DOCOPT_EXPORT
    #else
        #define DOCOPT_EXPORT __declspec(dllimport)   // i.e., a client of a shared library
    #endif
#else
    #define DOCOPT_EXPORT // Linux, Mac
#endif

#endif // DOCOPT_WINDOW_INCLUDE_H_
