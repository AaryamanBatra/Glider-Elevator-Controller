function func = Controller
% INTERFACE
%
%   sensors
%       .t          (time)
%       .theta      (pitch angle)
%       .phi        (elevator angle)
%
%   references
%       .theta      (reference pitch angle)
%
%   parameters
%       .tStep      (time step)
%       .phidotMax  (maximum elevator angular velocity)  
%       .symEOM     (nonlinear EOMs in symbolic form)
%       .numEOM     (nonlinear EOMs in numeric form)
%
%   data
%       .whatever   (yours to define - put whatever you want into "data")
%
%   actuators
%       .phidot     (elevator angular velocity)

% Do not modify this function.
func.init = @initControlSystem;
func.run = @runControlSystem;
end

%
% STEP #1: Modify, but do NOT rename, this function. It is called once,
% before the simulation loop starts.
%

function [data] = initControlSystem(parameters, data)

%
% Here is a good place to initialize things...
%

load('DesignProblem03_EOMs.mat');
% Parse the equations of motion.
f = symEOM.f;
% Define symbolic variables that appear in the equations of motion.
syms theta phi xdot ydot thetadot phidot
%Choose State
fsym=[f;thetadot;phidot];

%Equilibrium Points
%GetEquilibriumPoint(f,theta,phi,xdot,ydot)
[theta_e,phi_e,xdot_e,ydot_e] = GetEquilibriumPoint(f,0,0,6,0)
%pause
thetadot_e = 0;
phidot_e = 0;
xhat = [0; 0; 0; 0; 0];
eqstate = [xdot_e;ydot_e;thetadot_e;theta_e;phi_e]
eqinput = [phidot_e];
state = [xdot; ydot; thetadot; theta; phi];
input = [phidot];
%Linearization
A = double([vpa(subs(jacobian(fsym,state),[state; input],[eqstate; eqinput]))])
B = double(vpa(subs(jacobian(fsym,input),[state; input],[eqstate; eqinput])))
C = [0 0 0 1 0;0 0 0 0 1];
W = ctrb(A,B);

%Verfies System is Controllable
rank(W)-length(A);

%Define Gains Controller

Qc = eye(5)
Rc = eye(1)
K = lqr(A,B,Qc,Rc)

%Verifies if Controller is Asymptoticly Stable
ei = eig(A-B*K)

%Verifies System is Observable
O = obsv(A,C)
rank(obsv(A,C))-length(A);

%Define Gains Observer
 Qo = 3*eye(2)
 Ro = 2*eye(5);

L = lqr(A',C',inv(Ro),inv(Qo))'
%Verifies if Observer is Asymptoticly Stable
eig(A-L*C)


%Saving Variables
data.A = A;
data.B = B;
data.C = C;
data.K = K;
data.L = L;
data.h = parameters.tStep;
data.eqstate = eqstate;
data.eqinput = eqinput;
data.xhat = xhat;

%%This is stuff recently added
%data.zhat = 0;

% [actuators,data] = runControlSystem(sensors,references,parameters,data);

end

%
% STEP #2: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
%

function [actuators,data] = runControlSystem(sensors, references, parameters, data)

A = data.A;
B = data.B;
C = data.C;
K = data.K;
L = data.L;
h = data.h;

xhat = data.xhat;
eqstate = data.eqstate;
eqinput = data.eqinput;
theta = sensors.theta;
phi = sensors.phi;

output = [theta; phi];

y = output - C*[eqstate];%error
u =-K*xhat;%input
dxhat = A*xhat+B*u-L*(C*xhat-y);%this is obersever. State Estimate
data.xhat = xhat + h*dxhat; % Next step for state estimate.
actuators.phidot = u+eqinput;%input taking into account eqinput


end

% INPUTS:
% f is a symbolic description of the nonlinear EOMs
%   (theta, phi, xdot, ydot) is a guess at the equilibrium point
% OUTPUTS:
%   (theta, phi, xdot, ydot) is an equilibrium point near the guess
function [theta,phi,xdot,ydot] = GetEquilibriumPoint(f,theta,phi,xdot,ydot)
% Initial guess
x0 = [theta;phi;xdot;ydot];
% Symbolic states
syms theta phi xdot ydot thetadot real
% Symbolic inputs
syms phidot real
% Symbolic EOMs with thetadot=0 and phidot=0
g = subs(f,[thetadot,phidot],[0,0]);
% Numeric EOMs
vars = [theta;phi;xdot;ydot];
g = matlabFunction(g,'Vars',{vars});
% Find solution
xe = fsolve(g,x0);
% Parse solution
theta = xe(1);
phi = xe(2);
xdot = xe(3);
ydot = xe(4);
end
