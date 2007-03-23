#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Tools/CMCTool.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>

using namespace OpenSim;
using namespace std;

std::string transcode(const XMLCh *ch)
{
	char *buffer = XMLString::transcode(ch);
	string str = buffer;
	delete[] buffer;
	return str;
}

void PrintNode(const DOMNode *node, const string &spaces = "")
{
	const DOMNodeList *list = node->getChildNodes();
	std::cout << transcode(node->getNodeName()) << " (" << list->getLength() << " children)" << std::endl;
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
	std::cout << "Document URI = " << IO::GetFileNameFromURI(transcode(doc->getDocumentURI())) << std::endl;
#if 0
	DOMNode *node=doc->getDocumentElement()->getFirstChild();
	std::cout << "child = " << transcode(node->getNodeName()) << std::endl;
	DOMNode *after=doc->getDocumentElement()->removeChild(node);
	PrintDocument(doc);
	std::cout << node << ", " << after << std::endl;
	std::cout << "child = " << transcode(node->getNodeName()) << std::endl;
	delete after;
#endif
	PrintDocument(doc);
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
