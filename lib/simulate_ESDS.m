function [pos, vel, out_] = simulate_ESDS(DSHandle, x0, simOptions)
    [spaceDim, demoNum] = size(x0);
    for demoIt=1:demoNum
        it = 1;
        xCurr = x0(:,demoIt);
        
        % Energy tank paramenters
        if(isfield(simOptions,'sMax'))
            params.s_max = simOptions.sMax;
        else
            params.s_max = 0.5*dot(xCurr, xCurr);
        end
        params.ds = 0.1 * params.s_max;
        params.dz = 0.01;
        sOld = params.s_max;
        

        % Energy tank parameters
        params.s_min = 0.0;

        % Store data for plotting
        out_{demoIt}.beta(it)   = 1;
        out_{demoIt}.gamma(it)  = 1;
        out_{demoIt}.s(it) = sOld;
        out_{demoIt}.x(:,it) = xCurr;
        xPrec = 10*xCurr;
        
        while(it<simOptions.iterNum && norm(xCurr-simOptions.goal)>simOptions.tol && norm(xCurr-xPrec)>1e-5)
            xPrec = xCurr;
            
            dxLin    = -xCurr;
            dxNonLin = (1-exp(-simOptions.a*((xCurr.'*xCurr))))*DSHandle(xCurr);
         
            % Storage function
            p_ds = xCurr.'*dxNonLin;
            p_d  = dot(xCurr, xCurr);
    
            [~, beta, gamma, s] = tank_stabilization(p_ds, p_d, simOptions.dt, params, sOld);
            sOld = s;   
            params.s_max = (1-exp(-simOptions.a*dot(xCurr,xCurr)))*simOptions.sMax;
            
            % Scale velocity           
            dx = dxLin + gamma * dxNonLin;
            
            % Compute next position
            xCurr = xCurr + dx*simOptions.dt;

            it = it + 1;
            
            % Store data for plotting
            out_{demoIt}.beta(it)   = beta;
            out_{demoIt}.gamma(it)  = gamma;
            out_{demoIt}.s(it)      = sOld;
            out_{demoIt}.x(:,it)    = xCurr;
            out_{demoIt}.xd(:,it-1) = dx;
        end
        
        if(simOptions.plotResult)
            figure(1);
            hold on;
            plot(out_{demoIt}.x(1,1:1:end), out_{demoIt}.x(2,1:1:end), 'k','Linewidth', 3)
            plot(out_{demoIt}.x(1,end), out_{demoIt}.x(2,end), 'k.','Linewidth', 2,'MarkerSize', 50)
        end
        
        out_{demoIt}.xd(:,end+1) = zeros(spaceDim,1); % Add null velocity for consistency
        
        pos{demoIt} = out_{demoIt}.x;
        vel{demoIt} = out_{demoIt}.xd;
    end
end