% Compute the velocity root mean squared error of ESDS. 
% 
% Author: Matteo Saveriano
%
% Copyright (c) 2021 Matteo Saveriano, Dept. of Computer Science and 
% Digital Science Center, University of Innsbruck, 6020, Innsbruck,
% Austria, https://iis.uibk.ac.at
%

function [errRMSE, err2] = velocity_rmse(DSHandle, xDemo, vDemo, dt, sMax)
    % Energy tank paramenters
    params.s_max = sMax;
    params.ds = 0.1*params.s_max;
    params.dz = 0.01;
    params.s_min = 0.0;
    sOld = params.s_max;

    for i =1:length(xDemo)
        xCurr = xDemo(:,i);
        
        % Compute (unstable) DS velocity
        dxLin    = -xCurr;
        dxNonLin = (1-exp(-0.01*(xCurr.'*xCurr)))*DSHandle(xCurr);
         
        % Storage function
        p_ds = xCurr.'*dxNonLin;
        p_d  = dot(xCurr, xCurr);
    
        [~, ~, gamma, s] = tank_stabilization(p_ds, p_d, dt, params, sOld);
        sOld = s;    
        params.s_max = (1-exp(-0.01*dot(xCurr,xCurr)))*sMax;
            
        % Scale velocity to stabilize the motion
        vCurr = dxLin + gamma*dxNonLin;

        % Compute suqared errors
        err2(i) = (vDemo(:,i)-vCurr)'*(vDemo(:,i)-vCurr);
    end 
    % Return RMSE error
    errRMSE = sqrt(mean(err2)); 
end