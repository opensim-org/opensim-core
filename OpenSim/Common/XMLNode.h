#ifndef _XMLNode_h_
#define _XMLNode_h_
// XMLNode.h
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
#include <string>
#include "osimCommonDLL.h"
XERCES_CPP_NAMESPACE_USE


//using namespace std;	// Ayman:per .NET 2003



namespace OpenSim { 
//=============================================================================
//=============================================================================
/**
 * A class for performing operations on XML nodes.
 *
 * The methods in this class are generally static methods that take
 * an DOMNode as their first argument.
 *
 * This product includes software developed by the
 * Apache Software Foundation (http://www.apache.org/).
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
class OSIMCOMMON_API XMLNode {
//=============================================================================
// DATA
//=============================================================================
//	DOMNode *dumNode;
	static const int TABLIMIT;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	static int GetNumberOfParents(const DOMNode *node);
	static std::string ToString(const DOMNode *aNode);
	static std::string NodeContextString(const DOMNode *aNode);
	static std::string Transcode(const XMLCh *aCh);
	static std::string TranscodeAndTrim(const XMLCh *aCh);

	//--------------------------------------------------------------------------
	// ADD AND REMOVE NODES
	//--------------------------------------------------------------------------
	static DOMElement* 
		AppendNewElementWithComment(DOMNode *aParent,
							const std::string &aTag,
							const std::string &aName="",
							const std::string &aComment="");
	static DOMElement* 
		InsertNewElementWithComment(DOMNode *aParent,
							const std::string &aTag,
							const std::string &aName="",
							const std::string &aComment="",
							int aNodeIndex=0);
	static void
		RemoveChildren(DOMNode *aNode);
	static DOMNode*
		FindInsertionPoint(DOMNode *aElement, int aNodeIndex);
	static void
		RemoveElementFromParent(DOMElement *aElement, bool aRemoveWhitespaceAndComments = true);
	static void
		UpdateCommentNodeCorrespondingToChildElement(DOMElement *aElement,const std::string &aComment);
	static DOMText*
		CreateDOMText(DOMDocument *aDocument, const std::string &aText);
	static DOMComment*
		CreateDOMComment(DOMDocument *aDocument, const std::string &aComment);
	static DOMElement*
		CreateDOMElement(DOMDocument *aDocument, const std::string &aTag);

	//--------------------------------------------------------------------------
	// GET NODES
	//--------------------------------------------------------------------------
	static DOMElement*
		GetFirstChildElementByTagName(const DOMNode *aNode,
		const std::string &aTagName,const std::string *aName=0,bool aCheckForMultiple=true);
	static DOMCharacterData*
		GetCDataSectionNode(const DOMNode *aNode);
	static DOMText*
		GetTextNode(const DOMNode *aNode);

	//--------------------------------------------------------------------------
	// FUNDAMENTAL VALUE TYPES
	//--------------------------------------------------------------------------
	static bool
		GetBool(const DOMNode *aNode);
	static int
		GetInt(const DOMNode *aNode);
	static double
		GetDbl(const DOMNode *aNode);
	static std::string
		GetStr(const DOMNode *aNode);

	//--------------------------------------------------------------------------
	// FUNDAMENTAL ARRAY TYPES
	//--------------------------------------------------------------------------
	// BOOL ARRAY
	static void
		SetBoolArray(DOMNode *aNode,int aN,const bool *aData);
	static int
		GetBoolArray(const DOMNode *aNode,bool *&rData);
	// INT ARRAY
	static void
		SetIntArray(DOMNode *aNode,int aN,const int *aData);
	static int
		GetIntArray(const DOMNode *aNode,int *&rData);
	// DOUBLE ARRAY
	static void
		SetDblArray(DOMNode *aNode,int aN,const double *aData);
	static int
		GetDblArray(const DOMNode *aNode,double *&rData);
	// STRING ARRAY
	static void
		SetStrArray(DOMNode *aNode,int aN,const std::string *aData);
	static int
		GetStrArray(const DOMNode *aNode,std::string* &rData);

	//--------------------------------------------------------------------------
	// TEMPLATED VERSIONS
	//--------------------------------------------------------------------------
	template<class T> static T GetValue(const DOMNode *aNode);
	template<class T> static void SetValueArray(DOMNode *aNode,int aN,const T *rData);
	template<class T> static int GetValueArray(const DOMNode *aNode,T *&rData);

	//--------------------------------------------------------------------------
	// ELEMENT ATTRIBUTES
	//--------------------------------------------------------------------------
	static void
		SetAttribute(DOMNode *aNode,const std::string &aName,
		const std::string &aValue);
	static std::string
		GetAttribute(DOMNode *aNode,const std::string &aName);
	static void 
		RemoveAttribute(DOMNode *aNode,const std::string &aName);


//=============================================================================
};	// END CLASS XMLNode

// Template specializations
// Must be inline! (Trying to put function bodies in cpp fails with an internal compiler error in VC7.1)
template<> inline bool XMLNode::GetValue(const DOMNode *aNode) { return GetBool(aNode); }
template<> inline int XMLNode::GetValue(const DOMNode *aNode) { return GetInt(aNode); }
template<> inline double XMLNode::GetValue(const DOMNode *aNode) { return GetDbl(aNode); }
template<> inline std::string XMLNode::GetValue(const DOMNode *aNode) { return GetStr(aNode); }
template<> inline void XMLNode::SetValueArray(DOMNode *aNode,int aN,const bool *rData) { return SetBoolArray(aNode, aN, rData); }
template<> inline void XMLNode::SetValueArray(DOMNode *aNode,int aN,const int *rData) { return SetIntArray(aNode, aN, rData); }
template<> inline void XMLNode::SetValueArray(DOMNode *aNode,int aN,const double *rData) { return SetDblArray(aNode, aN, rData); }
template<> inline void XMLNode::SetValueArray(DOMNode *aNode,int aN,const std::string *rData) { return SetStrArray(aNode, aN, rData); }
template<> inline int XMLNode::GetValueArray(const DOMNode *aNode,bool *&rData) { return GetBoolArray(aNode, rData); }
template<> inline int XMLNode::GetValueArray(const DOMNode *aNode,int *&rData) { return GetIntArray(aNode, rData); }
template<> inline int XMLNode::GetValueArray(const DOMNode *aNode,double *&rData) { return GetDblArray(aNode, rData); }
template<> inline int XMLNode::GetValueArray(const DOMNode *aNode,std::string *&rData) { return GetStrArray(aNode, rData); }

}; //namespace
//=============================================================================
//=============================================================================

#endif // __XMLNode_h__
