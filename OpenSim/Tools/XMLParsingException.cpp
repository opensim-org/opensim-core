// XMLParsingException.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2007, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/


// INCLUDES
#include <iostream>
#include <string>
#include "XMLParsingException.h"
#include "IO.h"
#include "XMLNode.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
XMLParsingException::
XMLParsingException(const std::string &aMsg,const DOMNode *aNode,const std::string &aFile,int aLine)
	:Exception(aMsg,aFile,aLine)
{
	// NOTE: used to actually store the DOMNode as a member in this exception class
	// but it seems that it is not safe to manipulate the node once an exception is
	// thrown because its internal data becomes out of date.
	if(aNode) _xmlContextString = XMLNode::NodeContextString(aNode);
}

//=============================================================================
// IO
//=============================================================================
//-----------------------------------------------------------------------------
// PRINT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Print the exception to an output stream.
 *
 * @param aOut Output stream
 */
void XMLParsingException::
print(ostream &aOut)
{
	// HEADER
	aOut << "\nXMLParsingException:\n";

	// MESSAGE
	// Account for the _msg being multiple lines -- we want to prepend two spaces before each new line
	string formattedMsg = IO::formatText(_msg, "  ", 75);
	aOut << "  " << formattedMsg << endl;

	if(!_xmlContextString.empty())
		aOut << "  In:\n    " << IO::formatText(_xmlContextString, "    ", 75) << endl << endl;

	if(_file.size()>0) aOut << "  file= " << _file << '\n';
	if(_line>=0) aOut << "  line= " << _line << '\n';
	aOut << '\n';
}

