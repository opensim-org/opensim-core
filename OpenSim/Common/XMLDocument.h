#ifndef _XMLDocument_h_
#define _XMLDocument_h_
// XMLDocument.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include <iostream>	// Ayman: Remove .h extension per .NET 2003
#include <xercesc/dom/DOM.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/framework/XMLFormatter.hpp>
#include "osimCommonDLL.h"


XERCES_CPP_NAMESPACE_USE
//using namespace std;	// Ayman:per .NET 2003



namespace OpenSim { 
//=============================================================================
//=============================================================================
/**
 * A class for managing and for performing operations on an XML document.
 * The document can originate from one of two sources:  1) parsed from an
 * XML source or 2) created locally in memory.
 *
 * This product includes software developed by the
 * Apache Software Foundation (http://www.apache.org/).
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
class OSIMCOMMON_API XMLDocument {

//=============================================================================
// DATA
//=============================================================================
public:
	static const XMLCh UTF8[];
	static const XMLCh VERSION[];
	/** Latest version of the code encoded as an int xxyyzz where x: major release, y: minor, z: patch */
	static const int LatestVersion;
private:
	/** XML parser. */
	XercesDOMParser *_parser;
	/** XML document. */
	DOMDocument *_document;
	/** Name of the XML Document */
	std::string _fileName;
	/** Document Version as written to the file */
	int _documentVersion;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~XMLDocument();
	XMLDocument();
	XMLDocument(const std::string &aFileName);
	XMLDocument(const XMLDocument &aDocument);
	//--------------------------------------------------------------------------
	// VERSIONING /BACKWARD COMPATIBILITY SUPPORT
	//--------------------------------------------------------------------------	
	static const int& getLatestVersion() { return LatestVersion; };
	const int& getDocumentVersion() const { return _documentVersion; };
	static void getVersionAsString(const int aVersion, std::string& aString); 
	DOMElement* getRootDataElement();
private:
	void updateDocumentVersion();
	void setNull();

public:
	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
	DOMDocument* getDOMDocument() const;
	void setFileName(const std::string &aFileName);
	const std::string &getFileName() const;
	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	bool print(const std::string &aFileName=NULL);

private:
	static void CreateFormatter(std::ostream *aOstream=&std::cout);
	void printDeclaration();

//=============================================================================
};	// END CLASS XMLDocument

}; //namespace
//=============================================================================
//=============================================================================



// ---------------------------------------------------------------------------
//  Local classes
// ---------------------------------------------------------------------------

// Excluding this from Doxygen until it has better documentation! -Sam Hamner
    /// @cond
class DOMPrintFormatTarget : public XMLFormatTarget
{
	std::ostream *_out;
public:
	DOMPrintFormatTarget(std::ostream *aOStream= &std::cout)  {
		_out = aOStream;
	 };
    ~DOMPrintFormatTarget() {};

    // -----------------------------------------------------------------------
    //  Implementations of the format target interface
    // -----------------------------------------------------------------------

    void writeChars(const   XMLByte* const  toWrite,
                    const   unsigned int    count,
                            XMLFormatter * const formatter)
    {
        // Surprisingly, Solaris was the only platform on which
        // required the char* cast to print out the string correctly.
        // Without the cast, it was printing the pointer value in hex.
        // Quite annoying, considering every other platform printed
        // the string with the explicit cast to char* below.
        _out->write((char *) toWrite, (int) count);
    };

private:
    // -----------------------------------------------------------------------
    //  Unimplemented methods.
    // -----------------------------------------------------------------------
    DOMPrintFormatTarget(const DOMPrintFormatTarget& other);
    void operator=(const DOMPrintFormatTarget& rhs);
};
/// @endcond


#endif // __XMLDocument_h__
