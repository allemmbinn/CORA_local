function completed = example_f1Tenth_discrete
% example_f1Tenth - discrete time version of the example
% Syntax:
%    completed = example_nonlinearSysDT_reach_tank
% Inputs:
%    -
%
% Outputs:
%    completed - true/false 
%
% References:
%    [1] M. Althoff, “Reachability analysis and its application to the 
%        safety assessment of autonomous cars", Dissertation, 2010
%    [2] M. Althoff et al. "Reachability analysis of nonlinear systems with 
%        uncertain parameters using conservative linearization", CDC 2008

% Authors:       Allen Emmanuel Binny
% Written:       11-December-2023
% Last update:   ---
% Last revision: ---

% ------------------------------ BEGIN CODE -------------------------------

% Parameters --------------------------------------------------------------

params.tFinal = 3; %final time
% Initial set in terms of Center and Generators
params.R0 = zonotope([[0; 0; 0; 0; 0; 0; 0.2], diag([0.3; 0.1; 0; 0; 0.1; 0.1; 0])]);
% Control Input - Important to note that the steering velocity is 3.2
% Hence the U depends on the time discritisation
params.U = zonotope([[0; 0],diag([2;0.032])]);
% Input Function
params.u = 

% Reachability Settings ---------------------------------------------------

options.zonotopeOrder = 200; %zonotope order
options.tensorOrder = 2;
options.errorOrder = 1;
options.lagrangeRem.simplify = 'simplify';


% System Dynamics ---------------------------------------------------------

% sampling time
dt = 0.1;
fun = @(x,u) vehicle_kextend_7EqDT(x,u,dt);

veh = nonlinearSysDT('F1Tenth',fun,dt,7,2); % initialize vehicle system

% Specifications ----------------------------------------------------------

% original specs: |vx| <= 7, |delta| <= 0.42 should be fulfilled at all times
speclim1 = 7;
speclim2 = 0.41;

hs1 = halfspace([0 0 0 -1 0 0 0],-speclim1);
hs2 = halfspace([0 0 0 1 0 0 0],-speclim1);
hs3 = halfspace([0 0 0 0 0 0 -1],-speclim2);
hs4 = halfspace([0 0 0 0 0 0 1],-speclim2);
spec = specification({hs1,hs2,hs3,hs4},'unsafeSet');

% Reachability Analysis ---------------------------------------------------

tic
R = reach(veh, params, options,spec);
tComp = toc;
disp(['Computation Time of reachable set: ',num2str(tComp)]);


% Simulation --------------------------------------------------------------

simOpt.points = 60;
simRes = simulateRandom(veh, params, simOpt);


% Visualization -----------------------------------------------------------
 
figure; hold on; box on;
projDim = [1 2];
useCORAcolors("CORA:contDynamics")
    
% plot reachable sets
plot(R,projDim);
    
% plot initial set
plot(R.R0,projDim);
  
% plot simulation results     
plot(simRes,projDim,'Marker','.');

% label plot
xlabel('vx');
ylabel('vy');

% example completed
completed = true;

% ------------------------------ END OF CODE ------------------------------
