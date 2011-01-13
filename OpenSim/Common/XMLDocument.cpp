// XMLDocument.cpp
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


//-----------------------------------------------------------------------------
// INCLUDES
//-----------------------------------------------------------------------------
#include <fstream>	// Ayman: remove .h per .NET 2003
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/sax/SAXException.hpp>
#include <xercesc/dom/DOMException.hpp>
#include <xercesc/sax/ErrorHandler.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include "osimCommonDLL.h"
#include "XMLDocument.h"
#include "Exception.h"




using namespace OpenSim;
using namespace std;


//-----------------------------------------------------------------------------
// DECLARATIONS
//-----------------------------------------------------------------------------
static ostream& operator<<(ostream& aOStream,DOMNode *aNode);
static ostream& operator<<(ostream& aOStream,const XMLCh *aString);


//-----------------------------------------------------------------------------
// STATICS
//-----------------------------------------------------------------------------
static DOMPrintFormatTarget *_FormatTarget = NULL;
static XMLFormatter *_Formatter = NULL;


//-----------------------------------------------------------------------------
// CONSTANTS
//-----------------------------------------------------------------------------
const XMLCh XMLDocument::UTF8[] = {
	chLatin_U, chLatin_T, chLatin_F, chDash, chDigit_8, 0 };

const XMLCh XMLDocument::VERSION[] = {
	chDigit_1, chPeriod, chDigit_0, 0 };

static const XMLCh  gEndElement[] = {
	chOpenAngle, chForwardSlash, chNull };

static const XMLCh  gEndPI[] = {
	chQuestion, chCloseAngle, chNull };

static const XMLCh  gStartPI[] = {
	chOpenAngle, chQuestion, chNull };

static const XMLCh  gXMLDecl1[] = {
        chOpenAngle, chQuestion, chLatin_x, chLatin_m, chLatin_l
    ,   chSpace, chLatin_v, chLatin_e, chLatin_r, chLatin_s, chLatin_i
    ,   chLatin_o, chLatin_n, chEqual, chDoubleQuote, chDigit_1
    ,   chPeriod, chDigit_0, chNull };

static const XMLCh  gXMLDecl2[] = {
        chDoubleQuote, chSpace, chLatin_e, chLatin_n, chLatin_c
    ,   chLatin_o, chLatin_d, chLatin_i, chLatin_n, chLatin_g, chEqual
    ,   chDoubleQuote, chNull };

static const XMLCh  gXMLDecl3[] = {
        chDoubleQuote, chQuestion, chCloseAngle
    ,   chLF, chNull };

static const XMLCh  gStartCDATA[] = {
        chOpenAngle, chBang, chOpenSquare, chLatin_C, chLatin_D,
        chLatin_A, chLatin_T, chLatin_A, chOpenSquare, chNull };

static const XMLCh  gEndCDATA[] = {
    chCloseSquare, chCloseSquare, chCloseAngle, chNull };

static const XMLCh  gStartComment[] = {
    chOpenAngle, chBang, chDash, chDash, chNull };

static const XMLCh  gEndComment[] = {
    chDash, chDash, chCloseAngle, chNull };

static const XMLCh  gStartDoctype[] = {
    chOpenAngle, chBang, chLatin_D, chLatin_O, chLatin_C, chLatin_T,
    chLatin_Y, chLatin_P, chLatin_E, chSpace, chNull };

static const XMLCh  gPublic[] = {
    chLatin_P, chLatin_U, chLatin_B, chLatin_L, chLatin_I,
    chLatin_C, chSpace, chDoubleQuote, chNull };

static const XMLCh  gSystem[] = {
    chLatin_S, chLatin_Y, chLatin_S, chLatin_T, chLatin_E,
    chLatin_M, chSpace, chDoubleQuote, chNull };

static const XMLCh  gStartEntity[] = {
    chOpenAngle, chBang, chLatin_E, chLatin_N, chLatin_T, chLatin_I,
    chLatin_T, chLatin_Y, chSpace, chNull };

static const XMLCh  gNotation[] = {
    chLatin_N, chLatin_D, chLatin_A, chLatin_T, chLatin_A,
    chSpace, chDoubleQuote, chNull };

const int XMLDocument::LatestVersion = 20201;	
//=============================================================================
// DESTRUCTOR AND CONSTRUCTOR(S)
//=============================================================================
//-----------------------------------------------------------------------------
// DESTRUCTOR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Handle delete of an XMLDocument object.
 */
XMLDocument::~XMLDocument()
{
	// DELETE FORMATTER
	if(_Formatter!=NULL) {
		delete _Formatter;  _Formatter = NULL;
	}
	if(_FormatTarget!=NULL) {
		delete _FormatTarget;  _FormatTarget = NULL;
	}

	// DELETE PARSER AND DOCUMENT
	// Note that the document is deleted by the parser.
	if(_parser!=NULL) {
		delete _parser;  _parser=NULL;  _document=NULL;
	} else if(_document!=NULL) {
		_document->release(); _document=NULL;
	}

	// TERMINATE
	XMLPlatformUtils::Terminate();
}

//-----------------------------------------------------------------------------
// CONSTRUCTOR(S)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct a XMLDocument object with a locally generated DOMDocument.
 * This constructor is used when an XML document is going to be generated
 * locally in memory without reference to an XML file.  The initial
 * DOMDocument is empty.
 */
XMLDocument::XMLDocument()
{
	setNull();

	// INITIALIZE XML
	XMLPlatformUtils::Initialize();

	// MAKE AN EMPTY DOCUMENT
	DOMImplementation *implementation =
		DOMImplementation::getImplementation();
	_document = implementation->createDocument();
}

//_____________________________________________________________________________
/**
 * Construct an XMLDocument object from an XML document.
 * A parser is created for the purpose of reading in the XML file.
 *
 * @param aFileName File name of the XML document.
 */
XMLDocument::XMLDocument(const string &aFileName)
{
	setNull();

	// INITIALIZE XML
	XMLPlatformUtils::Initialize();

	// CREATE PARSER
	_parser = new XercesDOMParser();
	_parser->setErrorHandler(new HandlerBase());

	// PARSE DOCUMENT
	try {

		_parser->parse(aFileName.c_str());

	} catch (SAXException &x) {
		string msg =  "XMLDocument.XMLDocument: ERROR- SAX exception while attempting to parse " + aFileName;
		char *str = XMLString::transcode(x.getMessage());
		if(str) {
			msg += "\nSAXException message = " + string(str);
			delete[] str;
		}
		throw Exception(msg,__FILE__,__LINE__);
	} catch(...) {
		throw Exception("XMLDocument.XMLDocument: ERROR- exception while attempting to parse "+aFileName,__FILE__,__LINE__);
	}

/*
	// SAX EXCEPTION
	} catch (SAXException &x) {
		printf("XMLDocument.XMLDocument: ERROR- SAX exception while ");
		printf("attempting to parse %s.\n",aFileName);
		char *str = XMLString::transcode(x.getMessage());
		if(str!=NULL) {
			printf("%s",str);
			delete[] str;
		}
		return;

	// XML EXCEPTION
	} catch (XMLException &x) {
		printf("XMLDocument.XMLDocument: ERROR- XML exception while ");
		printf("attempting to parse %s.\n",aFileName);
		char *str = XMLString::transcode(x.getMessage());
		if(str!=NULL) {
			printf("%s",str);
			delete[] str;
		}
		return;

	// DOM EXCEPTION
	} catch (DOM_DOMException) {
		printf("XMLDocument.XMLDocument: ERROR- DOM exception while ");
		printf("attempting to parse %s.\n",aFileName);
		return;
	}
*/


	// DOCUMENT
	_document = _parser->getDocument();
	_fileName = aFileName;

	// Update document version based on parsing
	updateDocumentVersion();
}

//_____________________________________________________________________________
/**
 * Construct a copy of an XMLDocument object.  The document an all its nodes
 * are copied; however, the parser associated with the copied document, if
 * any, is not copied.
 */
XMLDocument::XMLDocument(const XMLDocument &aDocument)
{
	setNull();

	// INITIALIZE XML
	XMLPlatformUtils::Initialize();

	// CLONE
	DOMDocument *doc = aDocument.getDOMDocument();
	if(doc!=NULL) _document = (DOMDocument*)doc->cloneNode(true);

	_fileName = aDocument.getFileName();
}



//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void XMLDocument::
setNull()
{
	_parser = NULL;
	_document = NULL;
	_documentVersion = 10500;	// unless otherwise specified
}


//=============================================================================
// SET AND GET
//=============================================================================
//-----------------------------------------------------------------------------
// DOCUMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the XML document.
 *
 * Note that the returned document, and any of its child nodes, should never
 * be deleted by the caller.  The document and resources associated with
 * the document will be deleted when this XMLDocument object is deleted.
 *
 * @return Document, or NULL if no document has been generated.
 */
DOMDocument* XMLDocument::
getDOMDocument() const
{
	return(_document);
}

void XMLDocument::
setFileName(const string &aFileName)
{
	_fileName = aFileName;
}

const string &XMLDocument::
getFileName() const
{
	return _fileName;
}

//=============================================================================
// IO
//=============================================================================
//-----------------------------------------------------------------------------
// PRINT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Print the XML document to file.
 *
 * @param aFileName File name of the document to which to print
 */
bool XMLDocument::
print(const string &aFileName)
{
	if(_document==NULL) {
		printf("XMLDocument.print: WARN- no document.\n");
		return false;
	}

	// Standard Out
	if(aFileName.empty()) {
		CreateFormatter();
		printDeclaration();
		cout << _document;
		(*_Formatter) << chLF;
		cout << flush;

	// File
	} else {
		ofstream out(aFileName.c_str());
		if(out.good() == false) {
			printf("XMLDocument.print: Could not open file %s\n",
				aFileName.c_str());
			return false;
		}
		CreateFormatter(&out);
		printDeclaration();
		out << _document;
		(*_Formatter) << chLF;
		out << flush;
	}
	return true;
}
//_____________________________________________________________________________
/**
 * Print the XML document declaration.
 */
void XMLDocument::
printDeclaration()
{
	*_Formatter << gXMLDecl1;
	*_Formatter << gXMLDecl2 << UTF8;
	*_Formatter << gXMLDecl3;
}

//-----------------------------------------------------------------------------
// FORMATTER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Initialize a formatter for the document.
 * This method creates a formatter that is used by all documents for printing.
 */
void XMLDocument::
CreateFormatter(ostream *aOStream)
{
	// DELETE OLD
	if(_Formatter!=NULL) {
		delete _Formatter;  _Formatter = NULL;
	}
	if(_FormatTarget!=NULL) {
		delete _FormatTarget;  _FormatTarget = NULL;
	}

	// CONSTRUCT NEW
	_FormatTarget = new DOMPrintFormatTarget(aOStream);
	_Formatter = new XMLFormatter(UTF8,VERSION,_FormatTarget);
}

//-----------------------------------------------------------------------------
// STREAM OUTPUT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Output a node (DOMNode) and, recursively, all of its children to an
 * ouput stream.
 *
 * @param aOStream Output stream.
 * @param aNode IDOM node.
 */
ostream&
operator<<(ostream& aOStream,DOMNode *aNode)
{
    // Get the name and value out for convenience
    const XMLCh *   nodeName = aNode->getNodeName();
    const XMLCh *   nodeValue = aNode->getNodeValue();
    unsigned long lent = XMLString::stringLen(nodeValue);

    switch (aNode->getNodeType())
    {
        case DOMNode::TEXT_NODE:
        {
            _Formatter->formatBuf(nodeValue,
                                  lent, XMLFormatter::CharEscapes);
            break;
        }


        case DOMNode::PROCESSING_INSTRUCTION_NODE :
        {
            *_Formatter << XMLFormatter::NoEscapes << gStartPI  << nodeName;
            if (lent > 0)
            {
                *_Formatter << chSpace << nodeValue;
            }
            *_Formatter << XMLFormatter::NoEscapes << gEndPI;
            break;
        }


        case DOMNode::DOCUMENT_NODE :
        {

            DOMNode *child = aNode->getFirstChild();
            while( child != 0)
            {
                aOStream << child;
                // add linefeed in requested output encoding
                *_Formatter << chLF;
                aOStream << flush;

                child = child->getNextSibling();
            }
            break;
        }


        case DOMNode::ELEMENT_NODE :
        {
            // The name has to be representable without any escapes
            *_Formatter  << XMLFormatter::NoEscapes
                         << chOpenAngle << nodeName;

            // Output the element start tag.

            // Output any attributes on this element
            DOMNamedNodeMap *attributes = aNode->getAttributes();
            int attrCount = attributes->getLength();
            for (int i = 0; i < attrCount; i++)
            {
                DOMNode  *attribute = attributes->item(i);

                //
                //  Again the name has to be completely representable. But the
                //  attribute can have refs and requires the attribute style
                //  escaping.
                //
                *_Formatter  << XMLFormatter::NoEscapes
                             << chSpace << attribute->getNodeName()
                             << chEqual << chDoubleQuote
                             << XMLFormatter::AttrEscapes
                             << attribute->getNodeValue()
                             << XMLFormatter::NoEscapes
                             << chDoubleQuote;
            }

            //
            //  Test for the presence of children, which includes both
            //  text content and nested elements.
            //
            DOMNode *child = aNode->getFirstChild();
            if (child != 0)
            {
                // There are children. Close start-tag, and output children.
                // No escapes are legal here
                *_Formatter << XMLFormatter::NoEscapes << chCloseAngle;

                while( child != 0)
                {
                    aOStream << child;
                    child = child->getNextSibling();
                }

                //
                // Done with children.  Output the end tag.
                //
                *_Formatter << XMLFormatter::NoEscapes << gEndElement
                            << nodeName << chCloseAngle;
            }
            else
            {
                //
                //  There were no children. Output the short form close of
                //  the element start tag, making it an empty-element tag.
                //
                *_Formatter << XMLFormatter::NoEscapes << chForwardSlash << chCloseAngle;
            }
            break;
        }


        case DOMNode::ENTITY_REFERENCE_NODE:
            {
#if 0
                DOMNode *child;
                for (child = aNode->getFirstChild();
                child != 0;
                child = child->getNextSibling())
                {
                    aOStream << child;
                }
#else
                //
                // Instead of printing the refernece tree
                // we'd output the actual text as it appeared in the xml file.
                // This would be the case when -e option was chosen
                //
                    *_Formatter << XMLFormatter::NoEscapes << chAmpersand
                        << nodeName << chSemiColon;
#endif
                break;
            }


        case DOMNode::CDATA_SECTION_NODE:
            {
            *_Formatter << XMLFormatter::NoEscapes << gStartCDATA
                        << nodeValue << gEndCDATA;
            break;
        }


        case DOMNode::COMMENT_NODE:
        {
            *_Formatter << XMLFormatter::NoEscapes << gStartComment
                        << nodeValue << gEndComment;
            break;
        }


        case DOMNode::DOCUMENT_TYPE_NODE:
            {
            DOMDocumentType *doctype = (DOMDocumentType *)aNode;;

            *_Formatter << XMLFormatter::NoEscapes  << gStartDoctype
                        << nodeName;

            const XMLCh  *id = doctype->getPublicId();
            if (id != 0 && *id != 0)
            {
                *_Formatter << XMLFormatter::NoEscapes << chSpace << gPublic
                    << id << chDoubleQuote;
            }
            id = doctype->getSystemId();
            if (id != 0 && *id != 0)
            {
                *_Formatter << XMLFormatter::NoEscapes << chSpace
                    << chDoubleQuote << id << chDoubleQuote;
            }
            id = doctype->getSystemId();
            if (id != 0 && *id != 0)
            {
                *_Formatter << XMLFormatter::NoEscapes << chSpace << gSystem
                    << id << chDoubleQuote;
            }

            id = doctype->getInternalSubset();
            if (id != 0 && *id != 0)
                *_Formatter << XMLFormatter::NoEscapes << chOpenSquare
                << id << chCloseSquare;

            *_Formatter << XMLFormatter::NoEscapes << chCloseAngle;
            break;
        }


        case DOMNode::ENTITY_NODE:
        {
            *_Formatter << XMLFormatter::NoEscapes << gStartEntity
                        << nodeName;

            const XMLCh * id = ((DOMEntity *)aNode)->getPublicId();
            if (id != 0)
                *_Formatter << XMLFormatter::NoEscapes << gPublic
                            << id << chDoubleQuote;

            id = ((DOMEntity *)aNode)->getSystemId();
            if (id != 0)
                *_Formatter << XMLFormatter::NoEscapes << gSystem
                            << id << chDoubleQuote;

            id = ((DOMEntity *)aNode)->getNotationName();
            if (id != 0)
                *_Formatter << XMLFormatter::NoEscapes << gNotation
                            << id << chDoubleQuote;

            *_Formatter << XMLFormatter::NoEscapes << chCloseAngle << chLF;

            break;
        }



        default:
            cerr << "Unrecognized node type = "
                 << (long)aNode->getNodeType() << endl;
    }
    return aOStream;
}

//_____________________________________________________________________________
/**
 * Stream out an XML string (XMLCh).  The XMLCh string is converted to a
 * standard char* using XMLString::transcode().
 *
 * @param aOStream Output stream.
 * @param aString String.
 */
ostream&
operator<<(ostream& aOStream,const XMLCh *aString)
{
    char *p = XMLString::transcode(aString);
    aOStream << p;
    delete[] p;
    return aOStream;
}


//--------------------------------------------------------------------------
// VERSIONING /BACKWARD COMPATIBILITY SUPPORT
//--------------------------------------------------------------------------	
//_____________________________________________________________________________
/**
 * Convert passed in version number to a string 
 * The string is usually more compact e.g. 010100 -> 1_1 (rather than 11 which would confuse 110000 with 010100)
 */
void XMLDocument::
getVersionAsString(const int aVersion, std::string& aString)
{
	char pad[3];
	int ver = aVersion;
	aString = "";
	int div = 10000;
	for(int i=0; i<3; i++)
	{
		int digits = ver / div;
		sprintf(pad, "%02d",digits); 
		ver -= div*(ver / div);
		div /=100;
		aString += string(pad);
		if (ver ==0) break;
		aString +=(i<2?"_":"");
	}
}
//_____________________________________________________________________________
/**
 * Update member variable  _documentVersion based on parsing
 * Key assumption is that parsing has finished but no Object parsing is done yet
 */
void XMLDocument::
updateDocumentVersion()
{
	if (_document==NULL) return;
	// Check root node if it's OpenSimDocument
	DOMElement*     root = _document->getDocumentElement();
	const XMLCh *rootName=root->getNodeName();
	char *buffer = XMLString::transcode(rootName);
	if (strcmp(buffer, "OpenSimDocument")!=0){
		_documentVersion=10500;
		return;  // Old version pre 1.6
	}

	// Here we found a new root node of type OpenSimDocument
	// Locate attribute "Version" and extract value
	DOMNamedNodeMap* rootNodeAttributes=root->getAttributes();
	DOMNode  *versionAttribute= rootNodeAttributes->getNamedItem(XMLString::transcode("Version"));
	if (versionAttribute==NULL) {
		_documentVersion=10500;
		return; // Should assert here
	}
	const XMLCh*  versionString= versionAttribute->getNodeValue();
	buffer = XMLString::transcode(versionString);
	int readVersion;
	sscanf(buffer, "%d", &readVersion);
	// Validate >=  10600 and < latest as sanity check
	assert(readVersion >= 10600 && readVersion <= LatestVersion);
	_documentVersion = readVersion;
}
//_____________________________________________________________________________
/**
 * getRootDataElement returns a pointer to the real root node that contains objects 
 * works as a wrapper to get around the new root node <OpenSimDocument introduced in 1.6
 */
DOMElement*  XMLDocument::
getRootDataElement()
{
	if (_document==NULL) return NULL;
	// Check root node if it's OpenSimDocument
	DOMElement*     root = _document->getDocumentElement();
	if (_documentVersion < 10600) return root;  // Old version pre 1.6
	const XMLCh *rootName=root->getNodeName();
	char *buffer = XMLString::transcode(rootName);
	// Find first child with data and return it as new root should have exactly one
	// if we add more to the OpenSimDocument element e.g. CreationDate we should exclude it here
	DOMNodeList* immediateChildren = root->getChildNodes();
	int nodeIndexForObject=-1;
	for(int i=0; i<immediateChildren->getLength(); i++){
		DOMNode* nextElement = immediateChildren->item(i);
		const XMLCh *nodeName=nextElement->getNodeName();
		char* nodeText=XMLString::transcode(nextElement->getTextContent());
		if(nodeText) {
			XMLString::trim(nodeText);
			string str(nodeText);
			delete[] nodeText;
			if(str.length()>0)
				nodeIndexForObject=i;
		}
	}
	return (DOMElement*)immediateChildren->item(nodeIndexForObject);	

}
