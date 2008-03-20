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
// Written by Eran Guendelman, March 2007

#include <string>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/LoadModel.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <OpenSim/Simulation/Model/BodySet.h>

using namespace OpenSim;
using namespace std;

int main(int argc,char **argv)
{
	LoadOpenSimLibrary("osimActuators");
	LoadOpenSimLibrary("osimSdfastEngine");

	cout << endl << endl;
	Model *model1 = new Model("model1.osim");
	model1->setup();

	cout << endl << endl;
	Model *model2 = new Model("model2.osim");
	model2->setup();

	cout << endl << endl;

	int bodyidx = 1;

	cout << "Mass of model1:body1 = " << model1->getDynamicsEngine().getBodySet()->get(bodyidx)->getMass() << endl;
	cout << "Mass of model2:body1 = " << model2->getDynamicsEngine().getBodySet()->get(bodyidx)->getMass() << endl;

	cout << "\nSetting model1:body1 mass to 12345" << endl;
	model1->getDynamicsEngine().getBodySet()->get(bodyidx)->setMass(12345);

	cout << "Mass of model1:body1 = " << model1->getDynamicsEngine().getBodySet()->get(bodyidx)->getMass() << endl;
	cout << "Mass of model2:body1 = " << model2->getDynamicsEngine().getBodySet()->get(bodyidx)->getMass() << endl;

	cout << "\nSetting model2:body1 mass to 54321" << endl;
	model2->getDynamicsEngine().getBodySet()->get(bodyidx)->setMass(54321);

	cout << "Mass of model1:body1 = " << model1->getDynamicsEngine().getBodySet()->get(bodyidx)->getMass() << endl;
	cout << "Mass of model2:body1 = " << model2->getDynamicsEngine().getBodySet()->get(bodyidx)->getMass() << endl;
}
