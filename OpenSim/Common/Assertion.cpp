#include "Assertion.h"

#include <OpenSim/Common/Exception.h>

void OpenSim::OnAssertionError(
    char const* failingCode,
    char const* failingFile,
    char const* failingFunction,
    unsigned int failingLine,
    Object const* maybeSourceObject)
{
    if (maybeSourceObject) {
        throw Exception{failingFile, failingLine, failingFunction, *maybeSourceObject, failingCode};
    }
    else {
        throw Exception{failingFile, failingLine, failingFunction, failingCode};
    }
}