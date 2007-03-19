#ifndef _AnalysesDLL_h_
#define _AnalysesDLL_h_
// Analyses.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	Author:  Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// UNIX PLATFORM
#ifndef WIN32

#define OSIMANALYSES_API

// WINDOWS PLATFORM
#else

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#ifdef OSIMANALYSES_EXPORTS
#define OSIMANALYSES_API __declspec(dllexport)
#else
#define OSIMANALYSES_API __declspec(dllimport)
#endif

#endif // PLATFORM


#endif // __AnalysesDLL_h__
