%% Create Contact Geometries for OpenSim JAM tests
%==========================================================================
% This script requires GIBBON to be installed
% https://www.gibboncode.org/
%
%==========================================================================
clear
build_dir = 'C:\github\opensim-core-jam\build\OpenSim\Simulation\Test\';

% Ball
%------

nRefineSteps=5; %Number of refinement steps
sphereRadius=0.1; %Radius
closeOpt=0; %Option to close bottom of hemisphere

[F,V,C]=hemiSphereMesh(nRefineSteps,sphereRadius,closeOpt); %Construct hemi-shere mesh

V_rot(:,1) = V(:,1);
V_rot(:,2) = -V(:,3);
V_rot(:,3) = V(:,2);

TR = triangulation(F,V_rot);
% stlwrite(TR,'half_sphere_10cm_radius.stl')
% stlwrite(TR,[build_dir 'half_sphere_10cm_radius.stl'])
stlwrite('half_sphere_10cm_radius.stl',TR.ConnectivityList,TR.Points)
stlwrite([build_dir 'half_sphere_10cm_radius.stl'],TR.ConnectivityList,TR.Points)
sphereRadius=0.2; %Radius
closeOpt=0; %Option to close bottom of hemisphere

[F,V,C]=hemiSphereMesh(nRefineSteps,sphereRadius,closeOpt); %Construct hemi-shere mesh

V_rot(:,1) = V(:,1);
V_rot(:,2) = -V(:,3);
V_rot(:,3) = V(:,2);

TR = triangulation(F,V_rot);
% stlwrite(TR,'half_sphere_20cm_radius.stl')
% stlwrite(TR,[build_dir 'half_sphere_20cm_radius.stl'])
stlwrite('half_sphere_20cm_radius.stl',TR.ConnectivityList,TR.Points)
stlwrite([build_dir 'half_sphere_20cm_radius.stl'],TR.ConnectivityList,TR.Points)
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
% stlwrite(TR,'cylinder_10cm_radius.stl')
% stlwrite(TR,[build_dir 'cylinder_10cm_radius.stl'])
stlwrite('cylinder_10cm_radius.stl',TR.ConnectivityList,TR.Points)
stlwrite([build_dir 'cylinder_10cm_radius.stl'],TR.ConnectivityList,TR.Points)

inputStruct.cylRadius=0.2;

[F,V,C]=patchcylinder(inputStruct);
TR = triangulation(F,V);
% stlwrite(TR,'cylinder_20cm_radius.stl')
% stlwrite(TR,[build_dir 'cylinder_20cm_radius.stl'])
stlwrite('cylinder_20cm_radius.stl',TR.ConnectivityList,TR.Points)
stlwrite([build_dir 'cylinder_20cm_radius.stl'],TR.ConnectivityList,TR.Points)
% Plane
%------
clear F V;

 V1=[-0.5 0 -0.5; -0.5 0 0.5; 0.5 0 0.5; 0.5 0 -0.5];

regionCell={V1};
pointSpacing=0.01; %Desired point spacing
resampleCurveOpt=1;
interpMethod='linear'; %or 'natural'
[F,V]=regionTriMesh3D(regionCell,pointSpacing,resampleCurveOpt,interpMethod);
F=fliplr(F);
TR = triangulation(F,V);
% stlwrite(TR,'x_z_plane.stl')
% stlwrite(TR,[build_dir 'x_z_plane.stl'])
stlwrite('x_z_plane.stl',TR.ConnectivityList,TR.Points)
stlwrite([build_dir 'x_z_plane.stl'],TR.ConnectivityList,TR.Points)

% Disk
%-----
ns=500;
t=linspace(0,2*pi,ns);
t=t(1:end-1);
radius = 0.1;
[x,z] = pol2cart(t,radius*ones(size(t)));
y=zeros(size(x));
V2=[x(:) y(:) z(:)];

regionCell={V2};
pointSpacing=0.01; %Desired point spacing
resampleCurveOpt=1;
interpMethod='linear'; %or 'natural'
[F,V]=regionTriMesh3D(regionCell,pointSpacing,resampleCurveOpt,interpMethod);
% F=fliplr(F);
TR = triangulation(F,V);
% stlwrite(TR,'x_z_disk_10cm_radius.stl')
% stlwrite(TR,[build_dir 'x_z_disk_10cm_radius.stl'])
stlwrite('x_z_disk_10cm_radius.stl',TR.ConnectivityList,TR.Points)
stlwrite([build_dir 'x_z_disk_10cm_radius.stl'],TR.ConnectivityList,TR.Points)