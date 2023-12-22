function f = vehicle_kextend_7Eq_priority(x,u)
% vehicle_kextend_7EqDT - system dynamics for the 
% Extended Kinematic Model - We also apply the input accordingly
%
% Syntax:  
%    f = vehicle_kextend_7EqDT_priority(x,u)
%
% Inputs:
%    x - state vector
%    u - input vector
%
% Outputs:
%    f - new state vector
% 
% References:
%    [1] M. Althoff et al. "Reachability analysis of nonlinear systems with 
%        uncertain parameters using conservative linearization", CDC 2008

% Author:        Allen Emmanuel Binny
% Written:       20-Dec-2023
% Last update:   ---
% Last revision: ---

%------------- BEGIN CODE --------------

    % parameter
    lr = 0.17145;
    lf = 0.15875;

    % differential equations
    f(1,1) = (x(4)*cos(x(3)) - x(5)*sin(x(3)));           % x
    f(2,1) = (x(4)*sin(x(3)) + x(5)*cos(x(3)));           % y
    f(3,1) = (x(6));                                      % phi
    f(4,1) = (u(1));                                      % vx
    f(7,1) = (u(2));                                      % delta
    f(5,1) = (lr/(lf+lr) * (f(7,1)*x(4) + x(7)*f(4,1)));  % vy
    f(6,1) = (1/(lf+lr) * (f(7,1)*x(4) + x(7)*f(4,1)));   % w
                                      
%------------- END OF CODE --------------