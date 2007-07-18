// XMLNode.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include <xercesc/util/XMLStringTokenizer.hpp>
#include "osimCommonDLL.h"
#include "XMLNode.h"
#include "IO.h"
#include "Object.h"
#include "XMLParsingException.h"
#include <cassert>




using namespace OpenSim;
using namespace std;


// CONSTANTS
const int XMLNode::TABLIMIT = 128;


//=============================================================================
// UTILTITY
//=============================================================================

string WhitespaceString(int tabs,bool newline)
{
	char space[256];
	if(newline) {
		space[0]='\n';
		for(int i=1;i<tabs+1;i++) space[i]='\t';
		space[tabs+1]=0;
	} else {
		for(int i=0;i<tabs;i++) space[i]='\t';
		space[tabs]=0;
	}
	return string(space);
}

//_____________________________________________________________________________
/**
 * Get the number of paraents a node has.  The owner document is included
 * in this count.
 *
 * @param aNode Node for which to count the number of parents.
 * @return Number of parents including the owner document.  
 */
int XMLNode::
GetNumberOfParents(const DOMNode *aNode)
{
	if(aNode==NULL) return(0);

	int i=0;
	DOMNode *parent = aNode->getParentNode();
	while(parent!=NULL) {
		i++;
		parent = parent->getParentNode();
	}

	return(i);
}


//=============================================================================
// ADD AND REMOVE
//=============================================================================
//_____________________________________________________________________________
/**
 * Create a new element and add it as a child to a specified parent node with comment.
 * If comment needs to be specified but not name then "" should be passed for name.
 *
 * @param aParent Node to which to add the child element.
 * @param aTag Tag name of the element to be added.
 * @param aName Attribute name of the element.
 * @param aComment comment to be associated with xml node.
 * @return Child element.  NULL is returned on an error.
 */
DOMElement* XMLNode::
AppendNewElementWithComment(DOMNode *aParent,
							const string &aTag,
							const string &aName,
							const string &aComment)
{
	if(aParent==NULL) return(NULL);

	// GET DOCUMENT
	DOMDocument *doc;
	bool parentIsDoc = (aParent->getNodeType()==DOMNode::DOCUMENT_NODE);
	if(parentIsDoc) {
		doc = (DOMDocument*)aParent;
	} else {
		doc = aParent->getOwnerDocument();
	}
	if(doc==NULL) return(NULL);

	// DON'T ALLOW MORE THAN ONE ROOT
	if(parentIsDoc && (doc->getDocumentElement()!=NULL)) {
		printf("XMLNode.AddNewElement: ERROR- document already has root.\n");
		return(NULL);
	}

	// CREATE NEW NODE
	DOMElement *child = CreateDOMElement(doc,aTag);	
	if(!aName.empty()) SetAttribute(child,"name",aName);

	// DETERMINE LEVEL OF PARENT
	int level = GetNumberOfParents(aParent);
	if(level>TABLIMIT) level=TABLIMIT;

	// LEADING SPACE
	if(!parentIsDoc) {
		// If it already has children, we rely on the whitespace already there (just need to add a single tab)
		string space = aParent->hasChildNodes() ? WhitespaceString(1,false) : WhitespaceString(level,true);
		aParent->appendChild(CreateDOMText(doc,space));
	}

	// Add Comment if needed
	if (!aComment.empty()){
		// CREATE comment
		string space = WhitespaceString(level,false);
		string formattedComment = IO::formatText(aComment,space+"    ",70);
		aParent->appendChild(CreateDOMComment(doc,formattedComment));
		// Add newline
		aParent->appendChild(CreateDOMText(doc,WhitespaceString(level,true)));
	}

	// ADD CHILD
	if(child!=NULL) aParent->appendChild(child);

	// TRAILING SPACE
	if(!parentIsDoc) aParent->appendChild(CreateDOMText(doc,WhitespaceString(level-1,true)));

	return(child);
}
//_____________________________________________________________________________
/**
 * Remove all the children of a specified node.
 *
 * @param aNode Node whose children are to be removed.
 */
void XMLNode::
RemoveChildren(DOMNode *aNode)
{
	if(aNode==NULL) return;

	DOMNode *child;
	while((child = aNode->getFirstChild())) {
		aNode->removeChild(child);
	}

}
//_____________________________________________________________________________
/**
 * Remove an element from its parent, and optionally remove whitespace and comments
 * associated with that node.
 */
void XMLNode::
RemoveElementFromParent(DOMElement *aElement, bool aRemoveWhitespaceAndComments)
{
	if(!aElement) return;
	DOMNode *parent = aElement->getParentNode();
	if(parent) {
		// sanity check
		assert(parent->getNodeType() == DOMNode::ELEMENT_NODE || parent->getNodeType() == DOMNode::DOCUMENT_NODE);

		if(aRemoveWhitespaceAndComments) {
			// Find elements preceding and following this element
			DOMNode *prevElement = aElement->getPreviousSibling();
			while(prevElement && prevElement->getNodeType()!=DOMNode::ELEMENT_NODE) prevElement=prevElement->getPreviousSibling();
			DOMNode *nextElement = aElement->getNextSibling();
			while(nextElement && nextElement->getNodeType()!=DOMNode::ELEMENT_NODE) nextElement=nextElement->getNextSibling();

			DOMNode *first, *last;
			if(!prevElement && !nextElement) {
				// This is the last child element of this parent -- remove all child nodes
				first = parent->getFirstChild();
				last = parent->getLastChild();
			} else {
				// This is not the only element, so we remove all nodes from previous element (or from beginning) until this element.
				// This will get rid of any comments above this element.
				// There should be whitespace (text node) after this element that will make things line up properly.
				first = prevElement ? prevElement->getNextSibling() : parent->getFirstChild();
				last = aElement;
			}
			DOMNode *end=last->getNextSibling(), *next=0;
			for(DOMNode *cur=first; cur!=end; cur=next) {
				next = cur->getNextSibling();
				parent->removeChild(cur);
			}
		} else parent->removeChild(aElement);
	}
}

void XMLNode::
UpdateCommentNodeCorrespondingToChildElement(DOMElement *aElement,const std::string &aComment)
{
	// Look for a preceding comment node, or give up if reach the previous element
	DOMComment *commentNode = 0;
	for(DOMNode *child=aElement->getPreviousSibling(); child; child=child->getPreviousSibling()) {
		if(child->getNodeType()==DOMNode::COMMENT_NODE) {
			commentNode = (DOMComment*)child;
			break;
		}
		else if(child->getNodeType()==DOMNode::ELEMENT_NODE) break;
	}

	DOMDocument *doc = aElement->getOwnerDocument();
	if(!doc) return;

	// DETERMINE LEVEL OF PARENT
	int level = GetNumberOfParents(aElement)-1;
	if(level>TABLIMIT) level=TABLIMIT;

	string space = WhitespaceString(level,false);
	string formattedComment = IO::formatText(aComment,space+"    ",70);
	DOMComment *newCommentNode = CreateDOMComment(doc, formattedComment);

	if(commentNode) {
		aElement->getParentNode()->replaceChild(newCommentNode,commentNode);
	} else {
		aElement->getParentNode()->insertBefore(newCommentNode,aElement);
		DOMNode *spaceAfterCommentNode = CreateDOMText(doc, WhitespaceString(level,true));
		aElement->getParentNode()->insertBefore(spaceAfterCommentNode,aElement);
	}
}

DOMText* XMLNode::
CreateDOMText(DOMDocument *aDocument, const string &aText)
{
	XMLCh *xmlText = XMLString::transcode(aText.c_str());
	DOMText *node = aDocument->createTextNode(xmlText);
	delete[] xmlText;
	return node;
}

DOMComment* XMLNode::
CreateDOMComment(DOMDocument *aDocument, const string &aComment)
{
	XMLCh *xmlComment = XMLString::transcode(aComment.c_str());
	DOMComment *node = aDocument->createComment(xmlComment);
	delete[] xmlComment;
	return node;
}

DOMElement* XMLNode::
CreateDOMElement(DOMDocument *aDocument, const string &aTag)
{
	XMLCh *xmlTag = XMLString::transcode(aTag.c_str());
	DOMElement *node = aDocument->createElement(xmlTag);
	delete[] xmlTag;
	return node;
}

//=============================================================================
// GET NODES
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the first child of this node that is an element and has the specified
 * tag name (and, if aName is non-NULL, has the specified name attribute value).
 *
 * @param aNode Node that is to be search for the child element.
 * @param aTagName Tag name of the desired child element.
 * @param aName If non-NULL, specifies the name attribute value of the desired child element.
 * @return First child element node with a tag name aTagName.  NULL is
 * returned if no such element exists.
 */
DOMElement* XMLNode::
GetFirstChildElementByTagName(const DOMNode *aNode,const string &aTagName,const string *aName,bool aCheckForMultiple)
{
	if(aNode==NULL) return(NULL);

	// CONVERT TAG TO XMLCh
	XMLCh *tagName = XMLString::transcode(aTagName.c_str());
	if(tagName==NULL) return(NULL);

	XMLCh *nameAttrib = NULL, *name = NULL;
	if(aName) {
		nameAttrib = XMLString::transcode("name");
		name = XMLString::transcode(aName->c_str());
	}

	// LOOP THROUGH CHILDREN
	DOMElement *elmt = NULL;
	bool gotOne = false;
	for(DOMNode *child=aNode->getFirstChild(); child!=NULL;
										child=child->getNextSibling()) {

		if(child->getNodeType()!=DOMNode::ELEMENT_NODE) continue;
		DOMElement *e = (DOMElement*)child;
		if(XMLString::compareString(tagName,e->getTagName())==0 && 
			(!aName || XMLString::compareString(name,e->getAttribute(nameAttrib))==0)) 
		{
			if(aCheckForMultiple) {
				if(!gotOne) { gotOne = true; elmt = e; continue; }
				else {
					std::string msg = "Multiple instances of tag '"+aTagName + "'";
					if(aName) msg += " with name attribute '"+*aName+"'";
					throw XMLParsingException(msg,e,__FILE__,__LINE__);
				}
			} else {
				elmt = e;
				break;
			}
		}
	}

	// CLEANUP
	delete[] tagName;

	return(elmt);
}
//_____________________________________________________________________________
/**
 * Get the last character data section node associated with a specified node.
 *
 * @param aNode Node for which to identify a character data section node.
 * @return Character data section node. If aNode is a character data section
 * node it is returned.  Otherwise, aNode's children are searched, and the
 * LAST character data section node child found is returned.  If a character
 * data section node is not found, NULL is returned.
 */
DOMCharacterData* XMLNode::
GetCDataSectionNode(const DOMNode *aNode)
{
	DOMCharacterData *cdNode = NULL;

	// IS aNode A TEXT NODE?
	if(aNode->getNodeType()==DOMNode::CDATA_SECTION_NODE) {
		cdNode = (DOMCharacterData*)aNode;
		return(cdNode);
	}

	// LOOK FOR CHILD
	DOMNode *child;
	for(child=aNode->getFirstChild(); child!=NULL;
	                                  child=child->getNextSibling()) {

		if(child->getNodeType()==DOMNode::CDATA_SECTION_NODE) {
			cdNode = (DOMCharacterData*)child;
		}
	}

	return(cdNode);
}
//_____________________________________________________________________________
/**
 * Get the last text node of a specified node.
 *
 * @param aNode Node for which to identify a text node.
 * @return Text node. If aNode is a text node it is returned.  Otherwise,
 * aNode's children are searched, and the LAST text node child found is
 * returned.  If a text node is not found, NULL is returned.
 */
DOMText* XMLNode::
GetTextNode(const DOMNode *aNode)
{
	DOMText *textNode = NULL;

	// IS aNode A TEXT NODE?
	if((aNode->getNodeType()==DOMNode::TEXT_NODE) ||
		(aNode->getNodeType()==DOMNode::CDATA_SECTION_NODE)) {
		textNode = (DOMText*)aNode;
		return(textNode);
	}

	// LOOK FOR CHILD
	DOMNode *child;
	for(child=aNode->getFirstChild(); child!=NULL;
												child=child->getNextSibling()) {

		if(child->getNodeType()==DOMNode::TEXT_NODE) {
			textNode = (DOMText*)child;
		}
	}

	return(textNode);
}


//=============================================================================
// FUNDAMENTAL VALUE TYPES
//=============================================================================
//_____________________________________________________________________________
/**
 * Interpret the value of this node as an bool.
 *
 * If the specified node is a text node (DOMText) then it
 * itself is interpretted.  However, if the specified node is any other type
 * of node (for example, an element node (DOMElement)), the data is
 * interpretted from the specified node's last child that is a text
 * node.  If the specified node has no such child, an exception is thrown.
 *
 * @return Value.  If an error is encountered a false is returned.
 * @throws Exception if it was not possible to get a value.
 */
bool XMLNode::
GetBool(const DOMNode *aNode)
{
	if(aNode==NULL) {
		string msg = "XMLNode.GetBool: XML node is NULL.";
		throw( Exception(msg,__FILE__,__LINE__) );
	}

	// GET THE TEXT NODE
	DOMText *textNode = GetTextNode(aNode);
	if(textNode==NULL) {
		string msg = "XMLNode.GetBool: XML node has no text node.";
		throw( Exception(msg,__FILE__,__LINE__) );
	}

	// GET VALUE
	char *str = XMLString::transcode(textNode->getNodeValue());
	if(str==NULL) {
		string msg = "XMLNode.GetBool: XML text node is NULL.";
		throw( Exception(msg,__FILE__,__LINE__) );
	}

	// INTERPRET
	bool value;
	char *ptr = strstr(str,"true");
	if(ptr!=NULL) {
		value = true;
	} else {
		value = false;
	}

	// CLEANUP
	if(str!=NULL) { delete[] str; }

	return(value);
}
//_____________________________________________________________________________
/**
 * Interpret the value of this node as an int.
 *
 * If the specified node is a text node (DOMText) then it
 * itself is interpretted.  However, if the specified node is any other type
 * of node (for example, an element node (DOMElement)), the data is
 * interpretted from the specified node's last child that is a text
 * node.  If the specified node has no such child, an exception is thrown.
 *
 * @return Value.  If an error is encountered a 0 is returned.
 * @throws Exception if it was not possible to get a value.
 */
int XMLNode::
GetInt(const DOMNode *aNode)
{
	if(aNode==NULL) {
		string msg = "XMLNode.GetInt: XML node is NULL.";
		throw( Exception(msg,__FILE__,__LINE__) );
	}

	// GET THE TEXT NODE
	DOMText *textNode = GetTextNode(aNode);
	if(textNode==NULL) {
		string msg = "XMLNode.GetInt: XML node has no text node.";
		throw( Exception(msg,__FILE__,__LINE__) );
	}

	// GET VALUE
	char *str = XMLString::transcode(textNode->getNodeValue());

	// INTERPRET AS INT
	int value;
	int status = sscanf(str,"%d",&value);

	// ERROR CHECK
	if(status!=1) {
		string name = XMLString::transcode(aNode->getNodeName());
		string msg = "XMLNode.getInt: ERROR- failed to interpret value of ";
		msg += name;
		msg += " as an int.";
		if(str!=NULL) { delete[] str; }
		throw( Exception(msg,__FILE__,__LINE__) );
	}

	// CLEANUP
	if(str!=NULL) { delete[] str; }

	return(value);
}
//_____________________________________________________________________________
/**
 * Interpret the value of this node as a double.
 *
 * If the specified node is a text node (DOMText) then it
 * itself is interpretted.  However, if the specified node is any other type
 * of node (for example, an element node (DOMElement)), the data is
 * interpretted from the specified node's last child that is a text
 * node.  If the specified node has no such child, an exception is thrown.
 *
 * @return Value.  If an error is encountered 0.0 is returned.
 * @throws Exception if it was not possible to get a value.
 */
double XMLNode::
GetDbl(const DOMNode *aNode)
{
	if(aNode==NULL) {
		string msg = "XMLNode.GetDbl: XML node is NULL.";
		throw( Exception(msg,__FILE__,__LINE__) );
	}

	// GET THE TEXT NODE
	DOMText *textNode = GetTextNode(aNode);
	if(textNode==NULL) {
		string msg = "XMLNode.GetDbl: XML node has no text node.";
		throw( Exception(msg,__FILE__,__LINE__) );
	}

	// GET VALUE
	char *str = XMLString::transcode(textNode->getNodeValue());

	// INTERPRET AS INT
	double value;
	double status = sscanf(str,"%lf",&value);

	// ERROR CHECK
	if(status!=1)  {
		string name = XMLString::transcode(aNode->getNodeName());
		string msg = "XMLNode.GetDbl: ERROR- failed to interpret value of ";
		msg += name;
		msg += " as a double.";
		if(str!=NULL) { delete[] str; }
		throw( Exception(msg,__FILE__,__LINE__) );
	}

	// CLEANUP
	if(str!=NULL) { delete[] str; }

	return(value);
}
//_____________________________________________________________________________
/**
 * Interpret the value of this node as string.  Whitespace is trimmed before string
 * is returned.
 *
 * If the specified node is a text node (DOMText) then it
 * itself is interpretted.  However, if the specified node is any other type
 * of node (for example, an element node (DOMElement)), the data is
 * interpretted from the specified node's last child that is a text
 * node.  If the specified node has no such child, rData is set to NULL
 * and 0 is returned.
 *
 * Note that the caller is responsible for deleting the returned string.
 *
 * @param aNode Node from which to obtain the data.
 * @return Value.  If an error is encountered NULL is returned.
 * @throws Exception if it was not possible to get a value.
 */
string XMLNode::
GetStr(const DOMNode *aNode)
{
	if(aNode==NULL) {
		string msg = "XMLNode.GetStr: XML node is NULL.";
		throw( Exception(msg,__FILE__,__LINE__) );
	}

	// GET THE TEXT NODE
	DOMText *textNode = GetTextNode(aNode);
	if(textNode==NULL) {
		string msg = "XMLNode.GetStr: XML node has no text node.";
		throw( Exception(msg,__FILE__,__LINE__) );
	}

	// GET VALUE
	char *buffer = XMLString::transcode(textNode->getNodeValue());
	XMLString::trim(buffer);
	string str(buffer);
	delete[] buffer;
	return str;
}


//=============================================================================
// FUNDAMENTAL ARRAY TYPES
//=============================================================================
//-----------------------------------------------------------------------------
// BOOL ARRAY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the text of a node to represent an array of booleans.
 *
 * If the specified node is a text node (DOMText) then
 * it itself is modified.  However, if the specified node is any other type
 * of node (for example, an element node (DOMElement)), the data is set
 * on the specified node's last child that is a text node.
 * If the specified node has no such child, a new text node is
 * created and appended to the specified node as a child.
 *
 * @param aNode Node on which to set the text data.
 * @param aN Size of the specified data array.
 * @param aData Data array.
 */
void XMLNode::
SetBoolArray(DOMNode *aNode,int aN,const bool *aData)
{
	if(aNode==NULL) return;
	if(aData==NULL) return;

	// GET THE TEXT NODE
	DOMText *textNode = GetTextNode(aNode);

	// CREATE AN EMPTY STRING
	XMLCh *empty = XMLString::transcode("");
	XMLCh *space = XMLString::transcode(" ");

	// MAKE A NODE IF NECESSARY
	if(textNode==NULL) {
		DOMDocument *doc = aNode->getOwnerDocument();
		textNode = doc->createTextNode(empty);
		aNode->appendChild(textNode);
	}

	// CHECK CharacterData NODE
	if(textNode==NULL) {
		printf("XMLNode.SetIntArray: ERROR- unable to find or create ");
		printf("text node.\n");
		if(empty!=NULL) delete[] empty;
		if(space!=NULL) delete[] space;
		return;
	}

	// CLEAR THE NODE
	textNode->setData(empty);

	// SET DATA
	int i;
	char tmp[Object::NAME_LENGTH];
	XMLCh *data = NULL;
	for(i=0;i<aN;i++) {
		textNode->appendData(space);
		if(aData[i]==true) {
			sprintf(tmp,"true");
		} else {
			sprintf(tmp,"false");
		}
		data = XMLString::transcode(tmp);
		if(data==NULL) continue;
		textNode->appendData(data);
		delete[] data;
	}

	// ADD A SPACE ON END
	textNode->appendData(space);

	// CLEANUP
	if(empty!=NULL) delete[] empty;
	if(space!=NULL) delete[] space;
}
//_____________________________________________________________________________
/**
 * Interpret the value of this node as an array of boolean variables.
 *
 * If the specified node is a text node (DOMText) then it
 * itself is interpretted.  However, if the specified node is any other type
 * of node (for example, an element node (DOMElement)), the data is
 * interpretted from the specified node's last child that is a text
 * node.  If the specified node has no such child, rData is set to NULL
 * and 0 is returned.
 * The caller is responsible for deleting the returned array.
 *
 * @param aNode Node from which to obtain the data.
 * @param rData Reference to a bool pointer.  If there is no data, rData
 * is set to NULL.
 * @return Length of the array.
 */
int XMLNode::
GetBoolArray(const DOMNode *aNode,bool *&rData)
{
	if(aNode==NULL) {
		rData = NULL;
		return(0);
	}

	// GET THE TEXT NODE
	DOMText *textNode = GetTextNode(aNode);

	// TOKENIZE THE STRING
	XMLStringTokenizer tokenizer(textNode->getNodeValue());

	// COUNT
	int count = tokenizer.countTokens();
	if(count<=0) {
		rData = NULL;
		return(0);
	}

	// ALLOCATE SPACE
	rData = new bool[count];

	// LOOP OVER TOKENS
	XMLCh *tok;
	char *str;
	int n=0;
	while(tokenizer.hasMoreTokens()) {

		// NEXT TOKEN
		tok = tokenizer.nextToken();

		// CONVERT TO STRING
		str = XMLString::transcode(tok);
		if(str==NULL) continue;

		// INTERPRET
		if(stricmp("true",str)==0) {
			rData[n] = true;
		} else {
			rData[n] = false;
		}
		n++;

		// CLEANUP
		delete[] str;
	}

	return(n);
}

//-----------------------------------------------------------------------------
// INT ARRAY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the text of a node to represent an array of ints.
 *
 * If the specified node is a text node (DOMText) then
 * it itself is modified.  However, if the specified node is any other type
 * of node (for example, an element node (DOMElement)), the data is set
 * on the specified node's last child that is a text node.
 * If the specified node has no such child, a new text node is
 * created and appended to the specified node as a child.
 *
 * @param aNode Node on which to set the text data.
 * @param aN Size of the specified data array.
 * @param aData Data array.
 */
void XMLNode::
SetIntArray(DOMNode *aNode,int aN,const int *aData)
{
	if(aNode==NULL) return;
	if(aData==NULL) return;

	// GET THE TEXT NODE
	DOMText *textNode = GetTextNode(aNode);

	// CREATE AN EMPTY STRING
	XMLCh *empty = XMLString::transcode("");
	XMLCh *space = XMLString::transcode(" ");

	// MAKE A NODE IF NECESSARY
	if(textNode==NULL) {
		DOMDocument *doc = aNode->getOwnerDocument();
		textNode = doc->createTextNode(empty);
		aNode->appendChild(textNode);
	}

	// CHECK CharacterData NODE
	if(textNode==NULL) {
		printf("XMLNode.SetIntArray: ERROR- unable to find or create ");
		printf("text node.\n");
		if(empty!=NULL) delete[] empty;
		if(space!=NULL) delete[] space;
		return;
	}

	// CLEAR THE NODE
	textNode->setData(empty);

	// SET DATA
	int i;
	char tmp[Object::NAME_LENGTH];
	XMLCh *data = NULL;
	for(i=0;i<aN;i++) {
		textNode->appendData(space);
		sprintf(tmp,"%d",aData[i]);
		data = XMLString::transcode(tmp);
		if(data==NULL) continue;
		textNode->appendData(data);
		delete[] data;
	}

	// ADD A SPACE ON END
	textNode->appendData(space);

	// CLEANUP
	if(empty!=NULL) delete[] empty;
	if(space!=NULL) delete[] space;
}
//_____________________________________________________________________________
/**
 * Interpret the value of this node as an array of ints.
 *
 * If the specified node is a text node (DOMText) then it
 * itself is interpretted.  However, if the specified node is any other type
 * of node (for example, an element node (DOMElement)), the data is
 * interpretted from the specified node's last child that is a text
 * node.  If the specified node has no such child, rData is set to NULL
 * and 0 is returned.
 * The caller is responsible for deleting the returned array.
 *
 * @param aNode Node from which to obtain the data.
 * @param rData Reference to a int pointer.  If there is no data, rData
 * is set to NULL.
 * @return Length of the array.
 */
int XMLNode::
GetIntArray(const DOMNode *aNode,int *&rData)
{
	if(aNode==NULL) {
		rData = NULL;
		return(0);
	}

	// GET THE TEXT NODE
	DOMText *textNode = GetTextNode(aNode);

	// TOKENIZE THE STRING
	XMLStringTokenizer tokenizer(textNode->getNodeValue());

	// COUNT
	int count = tokenizer.countTokens();
	if(count<=0) {
		rData = NULL;
		return(0);
	}

	// ALLOCATE SPACE
	rData = new int[count];

	// LOOP OVER TOKENS
	XMLCh *tok;
	char *str;
	int status,n=0;
	while(tokenizer.hasMoreTokens()) {

		// NEXT TOKEN
		tok = tokenizer.nextToken();

		// CONVERT TO STRING
		str = XMLString::transcode(tok);
		if(str==NULL) continue;

		// INTERPRET
		status = sscanf(str,"%d",&rData[n]);
		if(status==1) n++;

		// CLEANUP
		delete[] str;
	}

	return(n);
}

//-----------------------------------------------------------------------------
// DOUBLE ARRAY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the text of a node to represent an array of doubles.
 *
 * If the specified node is a text node (DOMText) then
 * it itself is modified.  However, if the specified node is any other type
 * of node (for example, an element node (DOMElement)), the data is set
 * on the specified node's last child that is a text node.
 * If the specified node has no such child, a new text node
 * is created and appended to the specified node as a child.
 *
 * @param aNode Node on which to set the text data.
 * @param aN Size of the specified data array.
 * @param aData Data array.
 */
void XMLNode::
SetDblArray(DOMNode *aNode,int aN,const double *aData)
{
	if(aNode==NULL) return;
	if(aData==NULL) return;

	// GET THE TEXT NODE
	DOMText *textNode = GetTextNode(aNode);

	// CREATE AN EMPTY STRING
	XMLCh *empty = XMLString::transcode("");
	XMLCh *space = XMLString::transcode(" ");

	// MAKE A NODE IF NECESSARY
	if(textNode==NULL) {
		DOMDocument *doc = aNode->getOwnerDocument();
		textNode = doc->createTextNode(empty);
		aNode->appendChild(textNode);
	}

	// CHECK CharacterData NODE
	if(textNode==NULL) {
		printf("XMLNode.SetDblArray: ERROR- unable to find or create ");
		printf("text node.\n");
		if(empty!=NULL) delete[] empty;
		if(space!=NULL) delete[] space;
		return;
	}

	// GET DOUBLE OUTPUT FORMAT
	const char *format = IO::GetDoubleOutputFormat();

	// CLEAR THE NODE
	textNode->setData(empty);

	// SET DATA
	int i;
	char tmp[Object::NAME_LENGTH];
	XMLCh *data = NULL;
	for(i=0;i<aN;i++) {
		textNode->appendData(space);
		sprintf(tmp,format,aData[i]);
		data = XMLString::transcode(tmp);
		if(data==NULL) continue;
		textNode->appendData(data);
		delete[] data;
	}

	// ADD A SPACE ON END
	textNode->appendData(space);

	// CLEANUP
	if(empty!=NULL) delete[] empty;
	if(space!=NULL) delete[] space;
}
//_____________________________________________________________________________
/**
 * Interpret the value of this node as an array of doubles.
 *
 * If the specified node is a text node (DOMText) then it
 * itself is interpretted.  However, if the specified node is any other type
 * of node (for example, an element node (DOMElement)), the data is
 * interpretted from the specified node's last child that is a text
 * node.  If the specified node has no such child, rData is set to NULL
 * and 0 is returned.
 *
 * The caller is responsible for deleting the returned array.
 *
 * @param aNode Node from which to obtain the data.
 * @param rData Reference to a double pointer.  If there is no data, rData
 * is set to NULL.
 * @return Length of the array.
 */
int XMLNode::
GetDblArray(const DOMNode *aNode,double *&rData)
{
	if(aNode==NULL) {
		rData = NULL;
		return(0);
	}

	// GET THE TEXT NODE
	DOMText *textNode = GetTextNode(aNode);

	// TOKENIZE THE STRING
	XMLStringTokenizer tokenizer(textNode->getNodeValue());

	// COUNT
	int count = tokenizer.countTokens();
	if(count<=0) {
		rData = NULL;
		return(0);
	}

	// ALLOCATE SPACE
	rData = new double[count];

	// LOOP OVER TOKENS
	XMLCh *tok;
	char *str;
	int status,n=0;
	while(tokenizer.hasMoreTokens()) {

		// NEXT TOKEN
		tok = tokenizer.nextToken();

		// CONVERT TO STRING
		str = XMLString::transcode(tok);
		if(str==NULL) continue;

		// INTERPRET
		status = sscanf(str,"%lf",&rData[n]);
		if(status==1) n++;

		// CLEANUP
		delete[] str;
	}

	return(n);
}

//-----------------------------------------------------------------------------
// STRING ARRAY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the text of a node to represent an array of strings.
 *
 * If the specified node is a text node (DOMText) then
 * it itself is modified.  However, if the specified node is any other type
 * of node (for example, an element node (DOMElement)), the data is set
 * on the specified node's last child that is a text node.
 * If the specified node has no such child, a new text node
 * is created and added to the specified node as a child.
 *
 * @param aNode Node on which to set the text data.
 * @param aN Size of the specified data array.
 * @param aData Data array.
 */
void XMLNode::
SetStrArray(DOMNode *aNode,int aN,const string *aData)
{
	if(aNode==NULL) {
		return;
	}

	// GET THE TEXT NODE
	DOMText *textNode = GetTextNode(aNode);

	// CREATE AN EMPTY AND A SPACE XML STRING
	XMLCh *empty = XMLString::transcode("");
	XMLCh *space = XMLString::transcode(" ");

	// MAKE A NODE IF NECESSARY
	if(textNode==NULL) {
		DOMDocument *doc = aNode->getOwnerDocument();
		textNode = doc->createTextNode(empty);
		aNode->appendChild(textNode);
	}

	// CHECK CharacterData NODE
	if(textNode==NULL) {
		printf("XMLNode.SetStrArray: ERROR- unable to find or create ");
		printf("text node.\n");
		if(empty!=NULL) delete[] empty;
		if(space!=NULL) delete[] space;
		return;
	}

	// CLEAR THE NODE
	textNode->setData(empty);

	// SET DATA
	int i;
	XMLCh *data;
	for(i=0;i<aN;i++) {
		textNode->appendData(space);
		data = XMLString::transcode(aData[i].c_str());
		if(data==NULL) continue;
		textNode->appendData(data);
		delete[] data;
	}

	// ADD A SPACE ON END
	textNode->appendData(space);

	// CLEANUP
	if(empty!=NULL) delete[] empty;
	if(space!=NULL) delete[] space;
}
//_____________________________________________________________________________
/**
 * Interpret the value of this node as an array of strings.
 *
 * If the specified node is a text node (DOMText) then it
 * itself is interpretted.  However, if the specified node is any other type
 * of node (for example, an element node (DOMElement)), the data is
 * interpretted from the specified node's last child that is a text
 * node.  If the specified node has no such child, rData is set to NULL
 * and 0 is returned.
 *
 * The caller is responsible for deleting the individual strings in
 * the array as well as the array.
 *
 * @param aNode Node from which to obtain the data.
 * @param rData Reference to an array of character pointers.  If there
 * is no data, rData is set to NULL.
 * @return Length of the array.
 */
int XMLNode::
GetStrArray(const DOMNode *aNode,string* &rData)
{
	if(aNode==NULL) {
		rData = NULL;
		return(0);
	}

	// GET THE TEXT NODE
	DOMText *textNode = GetTextNode(aNode);

	// TOKENIZE THE STRING
	XMLStringTokenizer tokenizer(textNode->getNodeValue());

	// COUNT
	int count = tokenizer.countTokens();
	if(count<=0) {
		rData = NULL;
		return(0);
	}

	// ALLOCATE SPACE
	rData = new string[count];

	// LOOP OVER TOKENS
	XMLCh *tok;
	int n;
	for(n=0;tokenizer.hasMoreTokens();n++) {

		// NEXT TOKEN
		tok = tokenizer.nextToken();

		// CONVERT TO STRING
		char *str = XMLString::transcode(tok);
		rData[n] = str;
		if(str!=NULL) delete[] str;
	}

	return(n);
}


//=============================================================================
// ELEMENT ATTRIBUTES
//=============================================================================
//_____________________________________________________________________________
/**
 * Set an attribute on an element node.  This method only operates on element
 * nodes.  If the node sent in is not an element node, no action is taken.
 */
void XMLNode::
SetAttribute(DOMNode *aNode,const string &aName,const string &aValue)
{
	if(aNode==NULL) return;
	if(aNode->getNodeType()!=DOMNode::ELEMENT_NODE) return;

	// CAST NODE
	DOMElement *element = (DOMElement*)aNode;

	// CREATE NAME AND VALUE
	XMLCh *name = XMLString::transcode(aName.c_str());
	XMLCh *value = XMLString::transcode(aValue.c_str());

	// SET THE ATTRIBUTE
	element->setAttribute(name,value);

	// CLEANUP
	delete[] name;
	delete[] value;
}
//_____________________________________________________________________________
/**
 * Get an attribute of an element node.  This method only operates on element
 * nodes.  If the node sent in is not an element node, no action is taken.
 *
 * @param aNode Element node whose attribute is to be retrieved.
 * @param aName Name of the attribute.
 * @return Value of the attribute.  Note that xerces returns the empty string if an attribute does
 * not have a specified or default value.
 */
string XMLNode::
GetAttribute(DOMNode *aNode,const string &aName)
{
	if(aNode==NULL) return("");
	if(aNode->getNodeType()!=DOMNode::ELEMENT_NODE) return("");

	// CAST NODE
	DOMElement *element = (DOMElement*)aNode;

	// CREATE NAME
	XMLCh *name = XMLString::transcode(aName.c_str());

	// GET THE ATTRIBUTE
	const XMLCh *value = element->getAttribute(name);

	// CONVERT THE ATTRIBUTE INTO A STRING
	char *buffer = XMLString::transcode(value);
	assert(buffer);
	string str = buffer;

	// CLEANUP
	delete[] buffer;
	delete[] name;

	return(str);
}
/**
 * Remove attribute.
 */
void XMLNode::
RemoveAttribute(DOMNode *aNode,const string &aName)
{
	if(aNode==NULL) return;
	if(aNode->getNodeType()!=DOMNode::ELEMENT_NODE) return;

	// CAST NODE
	DOMElement *element = (DOMElement*)aNode;

	// CREATE NAME
	XMLCh *name = XMLString::transcode(aName.c_str());

	// REMOVE THE ATTRIBUTE
	if(element->hasAttribute(name)) element->removeAttribute(name);

	// CLEANUP
	delete[] name;
}
//=============================================================================
// UTILITY
//=============================================================================
string XMLNode::
ToString(const DOMNode *aNode)
{
	static const XMLCh *nameAttribute = XMLString::transcode("name");
	string str;
	if(aNode->getNodeType()==DOMNode::ELEMENT_NODE) {
		DOMElement *e=(DOMElement*)aNode;
		str = "<" + Transcode(e->getNodeName());
		string name = Transcode(e->getAttribute(nameAttribute));
		if(!name.empty()) str += " name=\"" + name + "\"";
		str += ">";
	} else {
		str += Transcode(aNode->getNodeName());
	}
	return str;
}

string XMLNode::
NodeContextString(const DOMNode *aNode)
{
	string str;
	const DOMNode *node = aNode;
	DOMDocument *doc = node->getOwnerDocument();
	int level = GetNumberOfParents(aNode);
	string spaces;
	for(int i=0;i<level;i++) spaces+=(i==0)?"+ ":"  ";
	while(node != doc) {
		if(!str.empty()) str = spaces + ToString(node) + "\n" + str;
		else str += spaces + ToString(node);
		node = node->getParentNode();
		spaces = spaces.substr(0,spaces.length()-2);
	}
	str = "file '" + IO::GetFileNameFromURI(Transcode(aNode->getOwnerDocument()->getDocumentURI())) + "'\n" + str;
	return str;
}

string XMLNode::
Transcode(const XMLCh *aCh)
{
	char *buffer = XMLString::transcode(aCh);
	if(buffer) {
		string str(buffer);
		delete[] buffer;
		return str;
	} else return string();
}

string XMLNode::
TranscodeAndTrim(const XMLCh *aCh)
{
	char *buffer = XMLString::transcode(aCh);
	if(buffer) {
		XMLString::trim(buffer);
		string str(buffer);
		delete[] buffer;
		return str;
	} else return string();
}

