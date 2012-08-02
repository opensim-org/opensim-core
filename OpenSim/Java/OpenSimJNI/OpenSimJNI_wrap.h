/* -------------------------------------------------------------------------- *
 *                        OpenSim:  OpenSimJNI_wrap.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#ifndef SWIG_opensimModel_WRAP_H_
#define SWIG_opensimModel_WRAP_H_

class SwigDirector_AnalysisWrapper : public OpenSim::AnalysisWrapper, public Swig::Director {

public:
    void swig_connect_director(JNIEnv *jenv, jobject jself, jclass jcls, bool swig_mem_own, bool weak_global);
    SwigDirector_AnalysisWrapper(JNIEnv *jenv, OpenSim::Model *aModel = 0);
    virtual ~SwigDirector_AnalysisWrapper();
    virtual OpenSim::AnalysisWrapper *clone() const;
    virtual std::string const &getConcreteClassName() const;
    virtual OpenSim::VisibleObject const *getDisplayer() const;
    virtual OpenSim::VisibleObject *updDisplayer();
    virtual void updateFromXMLNode(SimTK::Xml::Element &objectElement, int versionNumber);
    virtual void updateXMLNode(SimTK::Xml::Element &parent);
    virtual bool isA(char const *type) const;
    virtual void assign(OpenSim::Object &aObject);
    virtual int begin(SimTK::State &s);
    virtual int step(SimTK::State const &s, int stepNumber);
    virtual int end(SimTK::State &s);
    virtual void setModel(OpenSim::Model &aModel);
    virtual void setStatesStore(OpenSim::Storage const &aStatesStore);
    virtual bool proceed(int aStep = 0);
    virtual OpenSim::ArrayPtrs< OpenSim::Storage > &getStorageList();
    virtual int printResults(std::string const &aBaseName, std::string const &aDir = "", double aDT = -1.0, std::string const &aExtension = ".sto");
public:
    bool swig_overrides(int n) {
      return (n < 20 ? swig_override[n] : false);
    }
protected:
    bool swig_override[20];
};

class SwigDirector_SimtkLogCallback : public OpenSim::SimtkLogCallback, public Swig::Director {

public:
    void swig_connect_director(JNIEnv *jenv, jobject jself, jclass jcls, bool swig_mem_own, bool weak_global);
    SwigDirector_SimtkLogCallback(JNIEnv *jenv);
    virtual ~SwigDirector_SimtkLogCallback();
    virtual void log(std::string const &str);
public:
    bool swig_overrides(int n) {
      return (n < 1 ? swig_override[n] : false);
    }
protected:
    bool swig_override[1];
};


#endif
