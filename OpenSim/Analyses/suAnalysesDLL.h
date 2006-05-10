#ifndef _AnalysesDLL_h_
#define _AnalysesDLL_h_
// Analyses.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	Author:  Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// UNIX PLATFORM
#ifndef WIN32

#define SUANALYSES_API

// WINDOWS PLATFORM
#else

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#ifdef SUANALYSES_EXPORTS
#define SUANALYSES_API __declspec(dllexport)
#else
#define SUANALYSES_API __declspec(dllimport)
#endif

#endif // PLATFORM


#endif // __AnalysesDLL_h__
