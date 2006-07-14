// testDisplay1.cpp
// Author:  Ayman Habib
// Tests collection of Visible Objects and their dependencies.
#include <iostream>
#include <string>
#include <assert.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/Geometry.h>
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmModelVisibleIterator.h>
#include <OpenSim/Simulation/SIMM/SimmModelIterator.h>

using namespace OpenSim;
using namespace std;


// DECLARATIONS
int TestVisibleObjectCollection();
//_____________________________________________________________________________
/**
 * Test the rdTools library.
 */
int main(int argc, char* argv[])
{
	SimmModel* model = new SimmModel("C:/Downloads/MODEL/dynamic00.xml");
	SimmBody* pelvis = model->getSimmKinematicsEngine().getBody("pelvis");
	model->setup();
	pelvis = model->getSimmKinematicsEngine().getBody("pelvis");

	SimmModelVisibleIterator* it = new SimmModelVisibleIterator(*model);
	ArrayPtrs<VisibleObject>* visObjectsList= it->getVisibleObjects(0);
	for (int i=0; i < visObjectsList->getSize(); i++){
		VisibleObject *nextVisibleObject = visObjectsList->get(i);
		string name = nextVisibleObject->getName();
		string type = nextVisibleObject->getType();
		Object *owner = nextVisibleObject->getOwner();
		string ownerType = owner? owner->getType():"no-owner";
		
		cout << "Object:" << (owner?owner->getName():"no-owner") << "\t"  
			<< "#geo files+analytical" << (nextVisibleObject->countGeometry()) << "\t" 
			<< "Dependents" << nextVisibleObject->countDependents()
			<< endl;
	}
    SimmModelIterator *it2 = new SimmModelIterator(*model);

    SimmBody* gnd = model->getSimmKinematicsEngine().getGroundBodyPtr();
    while (it2->getNextBody() != 0) {

        SimmBody *body = it2->getCurrentBody();
		VisibleObject *bodyDisplayer = body->getDisplayer();
		int ct = bodyDisplayer->countDependents();
		for (int d=0; d < ct; d++){
			VisibleObject *Dependent = bodyDisplayer->getDependent(d);
			cout << Dependent->getOwner()->getType() << Dependent->getOwner()->getName() << endl;
		}
		for(int j=0; j < ct;j++){
			VisibleObject *Dependent = bodyDisplayer->getDependent(j);
			int geomcount = Dependent->countGeometry();
			// Create actor for the dpendent
			for(int gc=0; gc<geomcount; gc++){
				//vtkActor actor = new vtkActor();
				Geometry *g = const_cast<Geometry*> (Dependent->getGeometry(gc));
				AnalyticGeometry* ag = dynamic_cast<AnalyticGeometry*>(g);
				if (ag = (AnalyticGeometry*) g){
					AnalyticGeometry::AnalyticGeometryType analyticType = ag->getShape();
					Transform& trans = Dependent->getTransform();
					if (analyticType == AnalyticGeometry::AnalyticGeometryType::Sphere){
						/*
						System.out.println("Sphere for object "+Dependent.getOwner().getName());
						vtkSphereSource sphere = new vtkSphereSource();
						sphere.SetRadius(0.1);
						vtkPolyDataMapper mapper = new vtkPolyDataMapper();
						mapper.SetInput(sphere.GetOutput());
						actor.SetMapper(mapper);
						GetRenderer().AddViewProp(actor); 
						*/
					}
				}
			}
		}
	}
	return(0);
}
