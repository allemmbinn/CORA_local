% function completed = reach_f1Tenth_priority()

% example_nonlinear_reach_f1Tenth() - example of 
%     nonlinear reachability analysis
%
% Syntax:
%    completed = example_nonlinear_reach_05_autonomousCar()
%
% Inputs:
%    -
%
% Outputs:
%    completed - true/false.

% Authors:       Allen Emmanuel Binny
% Written:       20-December-2023
% Last update:   
% Last revision: ---

% ------------------------------ BEGIN CODE -------------------------------

% Parameters --------------------------------------------------------------
params.tFinal = 1.0;

% intial set
params.R0 = zonotope([[0; 0; 0; 0; 0 ; 0; 0],...
                      diag([0.3, 0.3, 0.001, 0.1, 0.1, 0.001, 0.001])]);

% uncertain inputs
params.U = zonotope([[0; 0],diag([0.01;0.001])]);

% Reachability Settings ---------------------------------------------------

options.timeStep = 0.05;
options.taylorTerms = 3;
options.zonotopeOrder = 100;

options.alg = 'lin';
options.tensorOrder = 2;


% System Dynamics ---------------------------------------------------------
vehicle = nonlinearSys(@vehicle_kextend_7Eq_priority,7,2);

% Specifications ----------------------------------------------------------
% original specs: |vx| <= 7, |delta| <= 0.41 should be fulfilled at all times
vlim = 7;
deltalim = -0.41;
alim = 7.5;
ratlim = deltalim/vlim;

hs1 = halfspace([0 0 0 -1 0 0 0],-vlim);
hs2 = halfspace([0 0 0 1 0 0 0],-vlim);
hs3 = halfspace([0 0 0 -ratlim 0 0 -1],-deltalim);
hs4 = halfspace([0 0 0 ratlim 0 0 1],-deltalim);
spec = specification({hs1,hs2,hs3,hs4},'unsafeSet');

% Determining the input values --------------------------------------------
inpU = repmat([5;0],1,20);
params.u = inpU;

% Obstacle ------------------
obst = [2,0,1,1]; %x,y,l,w
delchange = 0.01;
decchange = -0.2;
% Reachability Analysis ---------------------------------------------------
tic

% while(1)
%     params.u = inpU;
%     flag2 = 1;
%     R = reach(vehicle,params,options);
%     for i=2:length(R.timePoint.set)
%         Z1 = R.timePoint.set{i};
%         int1 = interval(Z1);
%         if(int1.sup(2)>obst(2) && int1.sup(1)>obst(1))
%             delinp = deltalim/(int1.sup(2)-int1.inf(2)) * (int1.sup(2)-obst(2));
%             if(abs(delinp) <= abs(inpU(2,i-1)) && abs(delinp) + delchange < deltalim)
%                 inpU(:,1:i-1) = inpU(:,1:i-1) + repmat([decchange;delchange],1,i-1);
%             elseif(abs(delinp) > abs(inpU(2,i-1)))
%                 inpU(:,1:i-1) = inpU(:,1:i-1) + repmat([decchange;delinp],1,i-1);
%             end
%             delinp
%             i
%             flag2 = 0;
%         elseif(int1.inf(2)<(obst(2)+obst(4)) && int1.sup(1)>obst(1))
%             delinp = deltalim/(int1.sup(2)-int1.inf(2)) * (int1.inf(2)-(obst(2)+obst(4)));
%             if(abs(delinp) <= abs(inpU(2,i-1)) && abs(delinp) + delchange < deltalim)
%                 inpU(:,1:i-1) = inpU(:,1:i-1) + repmat([decchange;-1*delchange],1,i-1);
%             elseif(abs(delinp) > abs(inpU(2,i-1)))
%                 inpU(:,1:i-1) = inpU(:,1:i-1) + repmat([decchange;delinp],1,i-1);
%             end
%             delinp
%             i
%             flag2 = 0;
%         end
%     end
%     if(flag2)
%         break;
%     end
% end
lastUpd = 0;
% while(1)
%     params.u = inpU;
%     flag2 = 1;
%     R = reach(vehicle,params,options);
%     for i=2:length(R.timePoint.set)
%         Z1 = R.timePoint.set{i};
%         int1 = interval(Z1);
%         if(int1.sup(2)>obst(2) && int1.sup(1)>obst(1))
%             delinp = deltalim/(int1.sup(2)-int1.inf(2)) * (int1.sup(2)-obst(2));
%             if(lastUpd == 0)
%                 inpU(:,i-1) = inpU(:,i-1)+ [decchange;delinp];
%                 lastUpd = i-1;
%             elseif(lastUpd>1)
%                 inpU(:,lastUpd-1) = inpU(:,lastUpd-1)+ [decchange;delinp];
%                 lastUpd = lastUpd -1;
%             end
%             delinp
%             i
%             flag2 = 0;
%         elseif(int1.inf(2)<(obst(2)+obst(4)) && int1.sup(1)>obst(1))
%             delinp = deltalim/(int1.sup(2)-int1.inf(2)) * (int1.inf(2)-obst(2)-obst(4));
%             if(lastUpd == 0)
%                 inpU(:,i-1) = inpU(:,i-1)+ [decchange;delinp];
%                 lastUpd = i-1;
%             elseif(lastUpd>1)
%                 inpU(:,lastUpd-1) = inpU(:,i-1)+ [decchange;delinp];
%             end
%             delinp
%             i
%             flag2 = 0;
%         end
%     end
%     if(flag2)
%         break;
%     end
% end
tComp = toc;
disp(['computation time of reachable set: ',num2str(tComp)]);

% % % Simulation --------------------------------------------------------------
% % simulation settings
% simOpt.points = 50;
% % random simulation
% simRes = simulateRandom(vehicle, params, simOpt);
% Visualization -----------------------------------------------------------
figure;  
hold on; box on
projDim = [1 2];

% plot reachable sets
useCORAcolors("CORA:contDynamics")
plot(R,projDim,'DisplayName','Reachable set');
    
% plot initial set
plot(R(1).R0,projDim, 'DisplayName','Initial set');
       
% Plot Obstacles
rectangle('Position', [2, 0, 1, 1], 'FaceColor', 'b');

% label plot    
xlabel('x');    
ylabel('y');    
legend()

% example completed
completed = true;
% ------------------------------ END OF CODE ------------------------------