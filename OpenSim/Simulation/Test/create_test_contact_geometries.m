%% Create Contact Geometries for OpenSim JAM tests
%==========================================================================
% This script requires GIBBON to be installed
% https://www.gibboncode.org/
%
%==========================================================================
clear
build_dir = 'C:\Users\csmith\github\jam-opensim-core\build\OpenSim\Simulation\Test\';

% Ball
%------

nRefineSteps=4; %Number of refinement steps
sphereRadius=0.1; %Radius
closeOpt=0; %Option to close bottom of hemisphere

[F,V,C]=hemiSphereMesh(nRefineSteps,sphereRadius,closeOpt); %Construct hemi-shere mesh

V_rot(:,1) = V(:,1);
V_rot(:,2) = -V(:,3);
V_rot(:,3) = V(:,2);

TR = triangulation(F,V_rot);
stlwrite(TR,'half_sphere_10cm_radius.stl')
stlwrite(TR,[build_dir 'half_sphere_10cm_radius.stl'])

sphereRadius=0.2; %Radius
closeOpt=0; %Option to close bottom of hemisphere

[F,V,C]=hemiSphereMesh(nRefineSteps,sphereRadius,closeOpt); %Construct hemi-shere mesh

V_rot(:,1) = V(:,1);
V_rot(:,2) = -V(:,3);
V_rot(:,3) = V(:,2);

TR = triangulation(F,V_rot);
stlwrite(TR,'half_sphere_20cm_radius.stl')
stlwrite(TR,[build_dir 'half_sphere_20cm_radius.stl'])

% Cylinder
%---------
clear F V;

% Creating input structure
inputStruct.cylRadius=0.1;
inputStruct.numRadial=100;
inputStruct.cylHeight=0.2;
inputStruct.numHeight=[]; %if empty infer from point spacing of circle
inputStruct.meshType='tri';
inputStruct.closeOpt=1;

[F,V,C]=patchcylinder(inputStruct);
TR = triangulation(F,V);
stlwrite(TR,'cylinder_10cm_radius.stl')
stlwrite(TR,[build_dir 'cylinder_10cm_radius.stl'])

inputStruct.cylRadius=0.2;

[F,V,C]=patchcylinder(inputStruct);
TR = triangulation(F,V);
stlwrite(TR,'cylinder_20cm_radius.stl')
stlwrite(TR,[build_dir 'cylinder_20cm_radius.stl'])

% Plane
%------
clear F V;

 V1=[-0.5 0 -0.5; -0.5 0 0.5; 0.5 0 0.5; 0.5 0 -0.5];

regionCell={V1};
pointSpacing=0.01; %Desired point spacing
resampleCurveOpt=1;
interpMethod='linear'; %or 'natural'
[F,V]=regionTriMesh3D(regionCell,pointSpacing,resampleCurveOpt,interpMethod);

TR = triangulation(F,V);
stlwrite(TR,'x_z_plane.stl')
stlwrite(TR,[build_dir 'x_z_plane.stl'])