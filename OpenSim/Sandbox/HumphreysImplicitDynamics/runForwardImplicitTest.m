function main

% Simulate dynamics similar to the OpenSim Tug-of-war model and
% performs a forward dynamic simulation.

%The following states are used through out:
%   y: s1; a1; s2; a2; x1; v1
%    ydot: sdot1; adot1; sdot2; adot2; vdot; vdotdot


clear all  
%because of the use of globals

import org.opensim.modeling.*

global  muscle
muscle = VandenBogert2011Muscle();

%Initial mass position
xInit=0;

% Make the control values
dt=0.001;
t=0:dt:1;

tPause=50*0.001;  %Start with 50 steps on constant excitation
nPause=fix(tPause/dt);
p(1,1:nPause)=0.2;

controlSig(1,:)=[p 0.2+0.1*(1+cos(1*2*pi*t+pi))];
controlSig(2,:)=0.2;

%Get the initial muscle lengths
muscleLengthInit=muscleLengthAndMomentArm(xInit);


% Initial Muscle Equilibrium  (we only need one muscle object because
%both muscle have the same parameters.
out=muscle.calcFiberStaticEquilbirum(muscleLengthInit(1), controlSig(1,1));
sInit1=out.get(0);
out=muscle.calcFiberStaticEquilbirum(muscleLengthInit(2), controlSig(2,1));
sInit2=out.get(0);

% Build the initial state vector
yInit=[sInit1;controlSig(1,1);sInit2;controlSig(2,1);0;0];
ydotInit=[0;0;0;0;0;0];




%Run forward implict
solver=1;  %0: IPOPT; 1:NR
useFiniteDiff=1;  %0: Exact Jacobian; 1: Finite Diff
yout=forwardImplicit(yInit,controlSig,dt,solver,useFiniteDiff);

%Plotting
figure
subplot(3,1,1)
plot(yout(1,:),'displayname','Muscle 1')
hold on
plot(yout(3,:),'displayname','Muscle 1')
ylabel('Projected Muscle Fiber (Norm)')
legend show

subplot(3,1,2)
plot(yout(2,:),'displayname','Muscle 1 Activation')
hold on
plot(yout(4,:),'displayname','Muscle 2 Activation')
plot(controlSig(1,:),'linestyle',':','displayname','Muscle 1 Excitation')
plot(controlSig(2,:),'linestyle',':','displayname','Muscle 2 Excitation')
ylabel('Excitation or Activation')
legend show

subplot(3,1,3)
plot(yout(5,:))
ylabel('Mass Position (m)')
xlabel('Step #')

end



%% ====================================================
%---------------Perform a forward simulation-------------------%
function [yout,ydotout]=forwardImplicit(yInit,controlSig,timeStep,solver,useFiniteDiffIn)

%Perform a forward dynamics like simulation.  Drive the system with
%muscle excitation and calculate positions.


global h u y useFiniteDiff
h=timeStep;

%Set the initial value
y =yInit;

%Set the initial guess to the initial value
ypGuess=yInit;


% %Check the Jacobians
% u=[0;0];
% useFiniteDiff=1;
% jFiniteDiff=ipoptConstraintsJacobian(yInit);
% useFiniteDiff=0;
% jExact=ipoptConstraintsJacobian(yInit);
% jacDiff=max((jFiniteDiff-jExact)./(abs(jFiniteDiff) + abs(jExact) ./2));
% if jacDiff>0.01
%     error('Exact and Finite Diff Jacobians do not seem to agree.')
% end

useFiniteDiff=useFiniteDiffIn;


%IPOPT Initial Parameters
options.lb=[0 0 0 0 -1 -Inf];
options.ub=[Inf 1 Inf 1 1 Inf];
options.cl=[0 0 0 0 0 0];
options.cu=[0 0 0 0 0 0];

funcs.objective         = @ipoptObj;
funcs.constraints       = @ipoptConstraints;
funcs.gradient          = @ipoptGradObj;
funcs.jacobian          = @ipoptConstraintsJacobian;
funcs.jacobianstructure = @() sparse(ones(6,6));

options.ipopt.jac_c_constant        = 'yes';
options.ipopt.hessian_approximation = 'limited-memory';
options.ipopt.mu_strategy           = 'adaptive';
options.ipopt.tol                   = 1e-5;
options.ipopt.print_level           = 0;


%Loop through the times steps to do forward solution
n=size(controlSig,2);
for i=1:n
    
    %Set the controls
    u=controlSig(:,i);
       
    if solver==0
        [yp, info] = ipopt(ypGuess,funcs,options);
    else
        dif=useFiniteDiff;
        [yp] = nrsolve(@implicitDynamics,dif,ypGuess);
    end

    %If other parameters are needed:
    %[residuals,muscleForces,massForce,yvec]=implicitDynamics(y2);
    
    %Store the results for later
    yout(:,i)=y;
    ydotout(:,i)=(yp-y)/h;
    
    
    ypGuess=yp;  %Make the next guess the current value
    y=yp;  %Make the current value the new starting point
    
    fprintf('Step %g of %g.\n',i,n)
    
end


end


%% ====================================================
%---------------Approximate the system dynamics-------------------%
function [residuals, df_dyp, muscleForces,massForce,yvec]=implicitDynamics(yp)

%Run implicit dynamics of the full system (this is a forward euler
%appoximation of the system.)  Note that "p" here is for prime, which means
%the guess or estimate.
%
%Inputs:
%   residuals - residuals of the implicit equation, row vector of:
%           [Force Res Muscle 1; Act Muscle 1; Force Res Muscle 2;
%               Act Res Muscle 2; Force Residual Body; Velocity Res Body
%
%Outputs:
%   df_dyp - Jacobian from yp to residuals
%   muscleForce - force in each muscle [N]
%   massForce - mass "acceleration" force [N]
%   yvec - large vector of [y;yp;ydot] for troubleshooting/plotting

global y u h

ypdot=(yp-y)/h;    %<-Forward Euler

%Give the model the current state, the guess of ydot, and the controls
[residuals,muscleForces,massForce, df_dy, df_dydot]=tug_of_war(y,ypdot,u);

% The system jacobian, J = df_dypdot = df_dydot * df_dydot/dypdot 
%                                                      ^--- = 1/h


df_dyp=df_dydot/h;   


yvec=[y;yp;ypdot];




end


%% ====================================================
%---------------muscle length and moments-------------------%
function [lm,r]=muscleLengthAndMomentArm(q)
%To book keep where moment arms will be needed
%   Inputs
%       q - joint position
%   Outputs
%       lm - muscle length
%       r - moment arm
lm(1)=q+0.3;   
r(1)=-1;  %moment arm = -dlm_dq

lm(2)=-q+0.3;
r(2)=1;
end

%% ====================================================
%---------------System Dynamics-------------------%
function [residuals, muscleForces, massForce, df_dy, df_dydot]=tug_of_war(y,ydot,u)
%Calculate the system dyanmics (the mass and muscles as a system)
%
%Inputs:
%   y - state vector of full system: 
%       [muscleLengthNorm1; activation1; muscleLengthNorm2; activation2;...
%               mass position; mass velocity]
%   ydot - state vector derivative
%   u - muscles excitation [excitation1; excitation2]
%
%Outputs:
%   residuals - implicit residuals:
%       [muscleForce1; muscleActivation1;muscleForce2; muscleActivation2;..
%           forceOnMass;massVeclocity]
%   muscleForce - force applied by each muscle
%   massForce - rigid body force on mass (accleration driven)
%   df_dy - Jacobian of residuals wrt to state y
%   df_dydot - Jacobian of residuals wrt to state ydot
%   

[muscleLength, momentArm]=muscleLengthAndMomentArm(y(5));

% Calculate the Residuals first for the 2 muscles and then the mass
[muscle1Res1, forceMuscle1, df_dy_Muscle1, df_dydot_Muscle1]=calcmuscle1Residual(y(1:2),ydot(1:2),u(1),muscleLength(1));
[muscle1Res2, forceMuscle2, df_dy_Muscle2, df_dydot_Muscle2]=calcmuscle1Residual(y(3:4),ydot(3:4),u(2),muscleLength(2));
[massRes, massForce, df_dy_Mass, df_dydot_Mass]=calcMassResidual(y(5:6),ydot(5:6),[forceMuscle1;forceMuscle2],momentArm);

residuals=[muscle1Res1;muscle1Res2;massRes];
muscleForces=[forceMuscle1;forceMuscle2];

%Assemble Jacobians
df_dy=zeros(6,6);
df_dydot=zeros(6,6);

df_dy(1:2,1:2)=df_dy_Muscle1;
df_dydot(1:2,1:2)=df_dydot_Muscle1;
df_dy(3:4,3:4)=df_dy_Muscle2;
df_dydot(3:4,3:4)=df_dydot_Muscle2;
df_dy(5:6,5:6)=df_dy_Mass;
df_dydot(5:6,5:6)=df_dydot_Mass;

end


%% ====================================================
%---------------Muscle Dynamics-------------------%
function [muscleRes,forceMuscle,df_dy,df_dydot]=calcmuscle1Residual(y,ydot,u,muscleLength)
%Calculate the muscle residuals. 
%
%Inputs:
%   y - state vector: [muscleLengthNorm; activation]
%   ydot - state vector derivative
%   u - muscle excitation
%   muscleLength - length of muscle [m]
%Outputs:
%   muscleRes - muscle residual (force;activation)
%   forceMuscle - force in Muscle [N]
%   df_dy - residual jacobian (residuals/y)
%   df_dydot - residual jacobian (residuals/ydot)


% Orginize variables for OpenSim API
import org.opensim.modeling.*
global muscle

y=Vec2(y(1),y(2));
ydot=Vec2(ydot(1),ydot(2));

%Run the Opensim Muscle code
out=muscle.calcImplicitResidual(y,ydot,muscleLength,u,1);

%Get the output
forceRes=out.getForceResidual;
actRes=out.getActivResidual;
muscleRes=[forceRes;actRes];

forceMuscle=out.getForceTendon;

[df_dy,muscle]=unFlattenMat22(out.getDf_dy(),muscle);  %Need to unhack this with Chris's fixed Mat22
[df_dydot,muscle]=unFlattenMat22(out.getDf_dydot(),muscle);  %Need to unhack this with Chris's fixed Mat22

end


%% ====================================================
%---------------Rigid Body Dynamics-------------------%
function [massRes,massForce,df_dy,df_dydot]=calcMassResidual(y,ydot,F,momentArm)
%% Calculate the mass residual
%Outputs
%   massRes - mass Residual, 2x1: [force residual; velocity residual]
%   massForce - rigid body force on mass
%   df_dy - Jacobian between residuals and y state vector
%   df_dydot - Jacobian between residuals and ydot state vector

%Inputs
%   y - state vector of [x;v];
%   ydot - state vector [xdot,vdotdot]
%   momentArms - 2x1 vector with muscle moment arms

m=10;
massForce = m*ydot(2);  %mass * acceleration

%Fres= muscle forces * moment arms - mass*accel 
forceRes= momentArm(1)*F(1)  + momentArm(2)*F(2)  - massForce;

velRes=y(2)-ydot(1);  % Velocity equation

massRes=[forceRes;velRes];

df_dy=[NaN NaN;NaN NaN];  %<<<< Not needed yet. Place holder
df_dydot=[0 -m; -1 0];

end

%% ====================================================
%---------------Jacobian by Finite Diff--------------%
function J=jacFiniteDiff(x,func,dx)

%Calculate the Jacobians by finite diff
%
%Inputs:
%   x - input vector
%   func - handle to function to calculate the jacobian of
%   dx- finite difference size
%
% Outputs:
%   J - Jacoboan df/dx (inputs in columns)

x=x(:);
m=length(x);

yo=func(x);
yo=yo(:)';

for i=1:m
    xTemp=x;
    xTemp(i)=xTemp(i)+dx;
    y=func(xTemp);
    y=y(:)';
    J(:,i)=(y-yo)/dx;
end

%J=J';
end

%% ====================================================
%----------------------NR Solver---------------------%
function [x,info] = nrsolve(func,useFiniteDiff,x,options)
% Solves f(x)=0 using the N-dimensional Newton-Raphson method
% with half-step backtracking
%
% Input:
%   func.............Function handle for F(x)
%   x................Initial guess
%   options..........Struct with solver options, can have these fields:
%       tolerance... Tolerance
%       print........Controls printing (0: none, 1: print everything)
%
% Output:
%   x................Solution


%set up toptions
if nargin<4  %Default options
    options.maxIter=100;
    options.tolerance=1e-4;
end

optFields=fieldnames(options);
for i=1:length(optFields)
    opt.(optFields{i})=options.(optFields{i});
end

% evaluate F(x) at the initial guess
[F,J] = func(x);
%fprintf('Initial guess: x=%f %f  F=%f %f\n',x,F);

% do the Newton iterations
for i = 1:opt.maxIter
    
    if useFiniteDiff
        %do finite diff jacobian
        h=1e-7;
        J=jacFiniteDiff(x,func,h);
    else
        [~,J] = func(x);   % and the Jacobian matrix dF/dx
        
    end
    
    dx = -J\F;              % solve dx from J*dx = -F
    newx = x + dx;          % do the Newton step
    [newF,newJ] = func(newx);
    if (norm(dx) < opt.tolerance)
        x=newx;
        break;          % if within tolerance, jump out of the "for"
    end
    %fprintf('   Iteration %d: dx=%10.6f %10.6f\n',i,dx);
    %fprintf('                 x=%10.6f %10.6f\n',newx);
    %fprintf('                 F=%10.6f %10.6f (norm %10.6f)\n',newF,norm(newF));
    
    % backtrack by half if the norm of F did not decrease during the step
    while norm(newF) >= norm(F)
        dx = dx/2;           % reduce the Newton step by half
        if norm(dx) < opt.tolerance
            error('step became too small during backtracking')
        end
        newx = x + dx;       % try the reduced step
        newF = func(newx);
        %fprintf('      Backtracking: dx=%10.6f %10.6f  norm(F)=%10.6f\n',dx,norm(newF));
    end
    
    % starting point for next iteration
    F = newF;
    x = newx;
end

% if we did the maximum number of iterations, we probably failed
if i == opt.maxIter;
    error('nrsolve: maximum number of iterations was done.')
end
end

%% ====================================================
%%--------   Functions needed by IPOPT ---------------%
function obj=ipoptObj(x)
%IPOPT Objective Function  (All 0)
obj=0;
end

function gradObj=ipoptGradObj(x)
%IPOPT Objectove Gradiant
gradObj=[1;1;1;1;1;1];
end

function constraints=ipoptConstraints(yp)
%IPOPT Contsraints Functions
%      Drive all residuals to 0
global y u h
residuals=implicitDynamics(yp);
constraints=residuals;
end

function constJac=ipoptConstraintsJacobian(yp)
%IPOPT Constraints Jacobian
global y u h useFiniteDiff
if useFiniteDiff
    df_dyp=jacFiniteDiff(yp,@implicitDynamics,1e-7);
else
    [~, df_dyp]=implicitDynamics(yp);
end
constJac=sparse(df_dyp);
end



