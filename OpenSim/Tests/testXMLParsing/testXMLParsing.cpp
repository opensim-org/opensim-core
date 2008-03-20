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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/XMLNode.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Tools/CMCTool.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>

using namespace OpenSim;
using namespace std;

void PrintSingleNode(const DOMNode *node)
{
	std::cout << XMLNode::Transcode(node->getNodeName()) << std::endl;
}

DOMElement *FindElement(const string &name, DOMNode *node)
{
	if(node->getNodeType() == DOMNode::ELEMENT_NODE && name == XMLNode::Transcode(node->getNodeName())) return (DOMElement*)node;
	else {
		const DOMNodeList *list = node->getChildNodes();
		for(unsigned int i=0;i<list->getLength();i++) {
			DOMElement *result = FindElement(name,list->item(i));
			if(result) return result;
		}
	}
	return 0;
}

void PrintNode(const DOMNode *node, const string &spaces = "")
{
	const DOMNodeList *list = node->getChildNodes();
	std::cout << XMLNode::Transcode(node->getNodeName()) << ": '" << XMLNode::Transcode(node->getNodeValue()) << "' (" << list->getLength() << " children)" << std::endl;
	for(unsigned int i=0;i<list->getLength();i++) {
		std::cout << spaces << "- " << i << ". ";
		PrintNode(list->item(i),spaces+"- ");
	}
}

void PrintDocument(const DOMDocument *doc)
{
	const DOMNode *node = doc->getDocumentElement();
	PrintNode(node);
}

int main(int argc,char **argv)
{
	if(argc<2) return 1;
	std::string filename = argv[1];

#if 1
	XMLDocument *xmldoc = new XMLDocument(filename);
	DOMDocument *doc = xmldoc->getDOMDocument();
	std::cout << "Document URI = " << IO::GetFileNameFromURI(XMLNode::Transcode(doc->getDocumentURI())) << std::endl;
#if 0
	DOMNode *node=doc->getDocumentElement()->getFirstChild();
	std::cout << "child = " << XMLNode::Transcode(node->getNodeName()) << std::endl;
	DOMNode *after=doc->getDocumentElement()->removeChild(node);
	PrintDocument(doc);
	std::cout << node << ", " << after << std::endl;
	std::cout << "child = " << XMLNode::Transcode(node->getNodeName()) << std::endl;
	delete after;
#endif
	PrintDocument(doc);

#if 0
	DOMElement *node = doc->createElement(XMLString::transcode("testing"));
	doc->getDocumentElement()->appendChild(node);
	doc->getDocumentElement()->removeChild(node);
	node->release();
	node->release();
//	delete node;
#endif

	DOMElement *el;

#if 0
	std::cout << std::endl << std::endl;
	el = FindElement("One",doc);
	XMLNode::RemoveElementFromParent(el,true);
	PrintDocument(doc);

	std::cout << std::endl << std::endl;
	el = FindElement("Two",doc);
	XMLNode::RemoveElementFromParent(el,true);
	PrintDocument(doc);

	std::cout << std::endl << std::endl;
	el = FindElement("Three",doc);
	XMLNode::RemoveElementFromParent(el,true);
	PrintDocument(doc);
#else
	std::cout << std::endl << std::endl;
	el = FindElement("Three",doc);
	XMLNode::RemoveElementFromParent(el,true);
	PrintDocument(doc);

	std::cout << std::endl << std::endl;
	el = FindElement("Two",doc);
	XMLNode::RemoveElementFromParent(el,true);
	PrintDocument(doc);

	std::cout << std::endl << std::endl;
	el = FindElement("One",doc);
	XMLNode::RemoveElementFromParent(el,true);
	PrintDocument(doc);
#endif

	doc->release();
//	delete doc;
#endif

#if 0
	Object::RegisterType(CMCTool());
	ControlSet *controlSet = new ControlSet(filename);
	for(int i=0;i<controlSet->getSize();i++) {
		std::cout << "Control " << i << ": " << controlSet->get(i)->getName() << std::endl;
		ControlLinear *linear = dynamic_cast<ControlLinear*>(controlSet->get(i));
		if(linear) {
			std::cout << "\tuse_steps = " << linear->getUseSteps() << std::endl;
			std::cout << "\tkp = " << linear->getKp() << std::endl;
			for(int j=0;j<linear->getNumParameters();j++) {
				std::cout << "\t" << linear->getParameterTime(j) << " : " << linear->getParameterValue(j) << std::endl;
			}
		}
	}
#endif
}
