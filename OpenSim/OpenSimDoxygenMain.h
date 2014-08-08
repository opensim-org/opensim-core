#ifndef _OpenSim_Doxygen_Main_h_
#define _OpenSim_Doxygen_Main_h_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  OpenSimDoxygenMain.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *                                                *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/** @file
This "header" file is actually just the source for OpenSim's Doxygen
Mainpage, the first page that a user sees when entering the Doxygen-
generated API documentation. This is not actually included as part of the
OpenSim source and it is not installed with OpenSim. **/

/** @mainpage  OpenSim 3.2 Documentation

\htmlonly
<!-- ImageReady Slices -->
The table below represents the conceptual heirarchy of OpenSim within SimTK. Each box represents a class utilized by the OpenSim API, and is built on top of (i.e., requires) the components underneath. 
For example, you can click on the "Model Component" block to see a list of model components, such as body, constraint, or joint.

<br/>
<br/> <b>Click on any of the boxes to navigate to corresponding class description.</b>
<table id="Table_01" width="800" height="411" border="0" cellpadding="0" cellspacing="0">
    <tr>
        <td colspan="15">
            <img src="images/DoxygenFrontPage_small_01.gif" width="800" height="2" alt=""></td>
    </tr>
    <tr>
        <td rowspan="10">
            <img src="images/DoxygenFrontPage_small_02.gif" width="2" height="408" alt=""></td>
        <td colspan="5" rowspan="2">
            <a href="classOpenSim_1_1Manager.html"
                onmouseover="window.status='Manager';  return true;"
                onmouseout="window.status='';  return true;">
                <img src="images/Manager.gif" width="272" height="98" border="0" alt="Manager"></a></td>
        <td colspan="5">
            <img src="images/DoxygenFrontPage_small_04.gif" width="265" height="1" alt=""></td>
        <td colspan="3" rowspan="2">
            <a href="classOpenSim_1_1Analysis.html"
                onmouseover="window.status='Analysis';  return true;"
                onmouseout="window.status='';  return true;">
                <img src="images/Analysis.gif" width="258" height="98" border="0" alt="Analysis"></a></td>
        <td rowspan="10">
            <img src="images/DoxygenFrontPage_small_06.gif" width="3" height="408" alt=""></td>
    </tr>
    <tr>
        <td rowspan="2">
            <img src="images/DoxygenFrontPage_small_07.gif" width="3" height="101" alt=""></td>
        <td colspan="3">
            <a href="https://simtk.org/api_docs/simbody/api_docs33/Simbody/html/classSimTK_1_1Optimizer.html"
                onmouseover="window.status='SimTK::Optimizer';  return true;"
                onmouseout="window.status='';  return true;">
                <img src="images/Optimizer.gif" width="259" height="97" border="0" alt="SimTK::Optimizer"></a></td>
        <td rowspan="2">
            <img src="images/DoxygenFrontPage_small_09.gif" width="3" height="101" alt=""></td>
    </tr>
    <tr>
        <td colspan="5">
            <img src="images/DoxygenFrontPage_small_10.gif" width="272" height="4" alt=""></td>
        <td colspan="3">
            <img src="images/DoxygenFrontPage_small_11.gif" width="259" height="4" alt=""></td>
        <td colspan="3">
            <img src="images/DoxygenFrontPage_small_12.gif" width="258" height="4" alt=""></td>
    </tr>
    <tr>
        <td rowspan="7">
            <img src="images/DoxygenFrontPage_small_13.gif" width="1" height="306" alt=""></td>
        <td colspan="2" rowspan="2">
            <a href="classOpenSim_1_1Model.html"
                onmouseover="window.status='Model';  return true;"
                onmouseout="window.status='';  return true;">
                <img src="images/Model_1.gif" width="195" height="48" border="0" alt="Model"></a></td>
        <td rowspan="2">
            <img src="images/DoxygenFrontPage_small_15.gif" width="3" height="48" alt=""></td>
        <td colspan="8">
            <a href="classOpenSim_1_1SimbodyEngine.html"
                onmouseover="window.status='Dynamics Engine';  return true;"
                onmouseout="window.status='';  return true;">
                <img src="images/Dynamics-Engine.gif" width="595" height="45" border="0" alt="Dynamics Engine"></a></td>
        <td rowspan="3">
            <img src="images/DoxygenFrontPage_small_17.gif" width="1" height="49" alt=""></td>
    </tr>
    <tr>
        <td colspan="8">
            <img src="images/DoxygenFrontPage_small_18.gif" width="595" height="3" alt=""></td>
    </tr>
    <tr>
        <td rowspan="2">
            <a href="classOpenSim_1_1Model.html"
                onmouseover="window.status='Model';  return true;"
                onmouseout="window.status='';  return true;">
                <img src="images/Model_1-20.gif" width="194" height="116" border="0" alt="Model"></a></td>
        <td colspan="5" rowspan="2">
            <a href="classOpenSim_1_1Model.html"
                onmouseover="window.status='Model';  return true;"
                onmouseout="window.status='';  return true;">
                <img src="images/Model_2.gif" width="164" height="116" border="0" alt="Model"></a></td>
        <td colspan="5">
            <img src="images/DoxygenFrontPage_small_21.gif" width="435" height="1" alt=""></td>
    </tr>
    <tr>
        <td rowspan="2">
            <img src="images/DoxygenFrontPage_small_22.gif" width="4" height="120" alt=""></td>
        <td colspan="5">
            <a href="classOpenSim_1_1ModelComponent.html"
                onmouseover="window.status='Model Component';  return true;"
                onmouseout="window.status='';  return true;">
                <img src="images/Model-Component.gif" width="432" height="115" border="0" alt="Model Component"></a></td>
    </tr>
    <tr>
        <td colspan="6">
            <img src="images/DoxygenFrontPage_small_24.gif" width="358" height="5" alt=""></td>
        <td colspan="5">
            <img src="images/DoxygenFrontPage_small_25.gif" width="432" height="5" alt=""></td>
    </tr>
    <tr>
        <td colspan="10">
            <a href="https://simtk.org/api_docs/simbody/api_docs33/Simbody/html/"
                onmouseover="window.status='SimTK::System';  return true;"
                onmouseout="window.status='';  return true;"
                target = "_blank">
                <img src="images/SimTK--System.png" width="792" height="134" border="0" alt="SimTK::System"></a></td>
        <td colspan="2" rowspan="2">
            <img src="images/DoxygenFrontPage_small_27.gif" width="2" height="137" alt=""></td>
    </tr>
    <tr>
        <td colspan="10">
            <img src="images/DoxygenFrontPage_small_28.gif" width="792" height="3" alt=""></td>
    </tr>
    <tr>
        <td>
            <img src="images/spacer.gif" width="2" height="1" alt=""></td>
        <td>
            <img src="images/spacer.gif" width="1" height="1" alt=""></td>
        <td>
            <img src="images/spacer.gif" width="194" height="1" alt=""></td>
        <td>
            <img src="images/spacer.gif" width="1" height="1" alt=""></td>
        <td>
            <img src="images/spacer.gif" width="3" height="1" alt=""></td>
        <td>
            <img src="images/spacer.gif" width="73" height="1" alt=""></td>
        <td>
            <img src="images/spacer.gif" width="3" height="1" alt=""></td>
        <td>
            <img src="images/spacer.gif" width="84" height="1" alt=""></td>
        <td>
            <img src="images/spacer.gif" width="4" height="1" alt=""></td>
        <td>
            <img src="images/spacer.gif" width="171" height="1" alt=""></td>
        <td>
            <img src="images/spacer.gif" width="3" height="1" alt=""></td>
        <td>
            <img src="images/spacer.gif" width="256" height="1" alt=""></td>
        <td>
            <img src="images/spacer.gif" width="1" height="1" alt=""></td>
        <td>
            <img src="images/spacer.gif" width="1" height="1" alt=""></td>
        <td>
            <img src="images/spacer.gif" width="3" height="1" alt=""></td>
    </tr>
</table>
<!-- End ImageReady Slices -->
<br/>
<b>Other specific classes of interest:</b>
<br/>
<a href = "classOpenSim_1_1Actuator.html">OpenSim::Actuator</a> <br/>
<a href = "classOpenSim_1_1Body.html">OpenSim::Body</a> <br/>
<a href = "classOpenSim_1_1Force.html">OpenSim::Force</a> <br/>
<a href = "classOpenSim_1_1Function.html">OpenSim::Function</a> <br/>
<a href = "classOpenSim_1_1Joint.html">OpenSim::Joint</a> <br/>
<a href = "classOpenSim_1_1Mtx.html">OpenSim::Mtx</a> <br/>
<a href = "classOpenSim_1_1Muscle.html">OpenSim::Muscle</a> <br/>
<br/>
<a href = "https://simtk.org/api_docs/simbody/api_docs33/Simbody/html/classSimTK_1_1Matrix__.html">SimTK::Matrix</a> <br/>
<a href = "https://simtk.org/api_docs/simbody/api_docs33/Simbody/html/classSimTK_1_1Vector__.html">SimTK::Vector</a> <br/>
<a href = "https://simtk.org/api_docs/simbody/api_docs33/Simbody/html/classSimTK_1_1Vec.html">SimTK::Vec</a> <br/>
<a href = "https://simtk.org/api_docs/simbody/api_docs33/Simbody/html/classSimTK_1_1UnitVec.html">SimTK::UnitVec</a> <br/>
<a href = "https://simtk.org/api_docs/simbody/api_docs33/Simbody/html/classSimTK_1_1Rotation__.html">SimTK::Rotation</a> <br/>
<a href = "https://simtk.org/api_docs/simbody/api_docs33/Simbody/html/classSimTK_1_1MobilizedBody.html">SimTK::MobilizedBody</a> <br/>
<a href = "https://simtk.org/api_docs/simbody/api_docs33/Simbody/html/classSimTK_1_1SimbodyMatterSubsystem.html">SimTK::SimbodyMatterSubsystem</a> <br/>
<a href = "https://simtk.org/api_docs/simbody/api_docs33/Simbody/html/classSimTK_1_1State.html">SimTK::State</a> <br/>

\endhtmlonly
**/

/** @page OpenSim_license_page  OpenSim Copyright and License

<h2>What is this?</h2>
You are looking at the website for the reference documentation of the OpenSim
API. This documentation was generated by Doxygen directly from the OpenSim 
source code as annotated by the original programmers so it is both accurate and
comprehensive. As you will quickly realize if you look around, this is a large 
system providing a great deal of functionality. It is, however, very easy to 
use if you know where to begin -- and this reference documentation is most
definitely \e not the place to start if you want to learn how to use 
OpenSim. Instead, start with the User's Guide, examples, and tutorials which
you can find at the OpenSim project site http://opensim.stanford.edu/support/index.html.
There is also a collection of working example programs that
come with the OpenSim installation (in the sdk directory). If you have 
specific questions, use the OpenSim user forum (under "Troubleshooting"). 
You can also file bug reports and make feature requests using the tools 
provided there.

<h2>Copyright and license</h2>
This license, based on the maximally-permissive Apache 2.0 License, defines the 
terms under which we offer OpenSim.
@verbatim
 * -------------------------------------------------------------------------- *
 *                              OpenSim: License                              *
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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- *
@endverbatim

<h2>How to Acknowledge Us</h2>
Acknowledging the OpenSim project helps us and helps you. It allows us to track our impact, which is essential for securing funding to improve the software and provide support to our users (you).
<br>
If you use the OpenSim GUI, you must acknowledge the OpenSim project where appropriate, by citing the following paper. If you only use the OpenSim API, you are not required to acknowledge us, but we would be extremely grateful if you did so anyway.
<br>
<br>
<i>Delp SL, Anderson FC, Arnold AS, Loan P, Habib A, John CT, Guendelman E, Thelen DG. OpenSim: Open-source Software to Create and Analyze Dynamic Simulations of Movement. IEEE Transactions on Biomedical Engineering. (2007)</i>
<br>
<br>
We would also be grateful if you mention that OpenSim is funded by NIH Roadmap grant U54 GM072970, the NIH research infrastructure grant R24 HD065690, and the DARPA Warrior Web Program. We greatly appreciate this support, and the NIH and DARPA appreciate knowing that their funds are having an impact, particularly on medical research and human health.
If you use plugins, models, or other components contributed by your fellow researchers, you must acknowledge their work as described in the license that accompanies each of these files.
<br>
<br>
<h2>Author List</h2>
The following people have contributed to the OpenSim API:
<br>
<br>
Frank C. Anderson, Allison S. Arnold, Scott L. Delp, Matt S. DeMers, Tim Dorn, Brian Garner, Saryn R. Goldberg, Eran Guendelman, Ayman Habib, Samuel R. Hamner, Jennifer L. Hicks, Katherine R. S. Holzbaur, Chand T. John, Cassidy Kelly, May Q. Liu, Peter Loan, Jack Middleton, Matthew Millard, Paul C. Mitiguy, Jeffrey A. Reinbolt, Ajay Seth, Michael A. Sherman, Darryl G. Thelen, Kevin Xu
**/

#endif // _OpenSim_Doxygen_Main_h_
