% Implement energy tank-based stabilization by modifying the formulation in 
%   Kronander and Billard, "Passive Interaction Control with Dynamical
%   Systems", RA-L, 2017.
%
% INPUT
%      z: Component of the storage function with undetermined sign
%    p_d: Positive-definite term of the storage function, i.e. the power
%         dissipated by the system
%     dt: Sampling time
%  params.s_max: Maximum allowed stored energy
%  params.ds, params.dz: Parameters to control the smoothness of the functions 
%  s_old:  State of the stored energy in previous step
%
% OUTPUT
%  gamma: Energy scaling action to passify the system
%      s:  State of the stored energy
   
function [alpha, beta, gamma, s] = tank_stabilization(z, p_d, dt, params, s_old)
    s = s_old;
    
    s_max = params.s_max;
    ds    = params.ds;
    dz    = params.dz;
    s_min = params.s_min;
    
    % Non-conservative energy change
    gamma = smooth_rise_2d(z,  s, 0, dz, 0, ds);

    % Update storage
    beta  = smooth_rise_fall_2d(z,s, 0, 0, dz, s_min, s_max, ds);
    alpha = smooth_rise_fall(s, 0, ds, s_max-ds, s_max);

    max(beta,0);
    if(z>=0)
        gamma = beta;
    end
           
    if(s>s_max) % Max s goes to zero for stability (s(x=0) = 0)
        % I do not use sdot = d k(||x||)/dt * smax because I do not need 
        % sdot. From this point I want that s has the same dynamics of
        % s_max and converges to zero.
        s = s_max;
    else
        alpha = min(0.99, alpha); % Guarantee alpha < 1 for stability
        sdot = alpha*p_d - beta*z;

        s = s + sdot*dt;
    end
end


%% Compute the smooth functions used for energy tank-based passification

% Function returning 0 for val<l0 and val>h0, 1 for l1<val<h1, and again
% 0 for val>h0
function h = smooth_rise_fall(val, l0, l1, h1, h0)
        hr = smooth_rise(val, l0, l1);
        hf = smooth_fall(val, h1, h0);
        h = hr*hf;
end

%%
% Function returning 0 for a smooth function that is zero for all x,y such
% that x<xhi and y>yhi, zero for all x,y, such that x>xlo and y<ylo, and
% 1 elsewhere. The transition is smooth and the output goes exactly to
% zero at the specified boundaires. dx and dy control smoothness, large
% value =>  smooth result
function h = smooth_rise_fall_2d(x, y, xlo, xhi, dx, ylo, yhi, dy)
    h1_1 = smooth_rise(x, xlo-dx, xlo);
    h1_2 = smooth_fall(y, ylo, ylo+dy);
    h1 = h1_1*h1_2;
    
    h2_1 = smooth_fall(x, xhi, xhi+dx);
    h2_2 = smooth_rise(y, yhi-dy, yhi);
    h2 = h2_1*h2_2;
    
    h = 1 - h1 - h2;
end

%%
% Function returning exactly 0 for x >= xlo and y =<ylo and >0  elsewhere.
% Smoothness controlled by dx and dy
function h = smooth_rise_2d(x, y, xlo, dx, ylo, dy)
    h1 = smooth_rise(x, xlo, xlo+dx);
    h2 = smooth_fall(y, ylo, ylo+dy);
    
    h = 1 - h1*h2;
end

%%
% Smooth step function returnning exactly 0.0 for val<lo and exactly 1.0 
% for val>1 and smooth transition in between
function smooth_value = smooth_rise(val, lo, hi)
    if(lo>hi)
        disp('Error in smooth_rise(val, lo, hi): lo > hi');
        smooth_value = -1;
        return;
    end
    
    if(val>=hi)
        smooth_value = 1;
    elseif(val<lo)
        smooth_value = 0;
    else
        T = 2*(hi-lo);
        smooth_value = 0.5+0.5*sin(2*pi*(val-lo)/T - pi*0.5);
    end
end

%%
% Smooth step function returning exactly 1.0 for val<hi and exactly 0.0
% for val>lo and smooth transition in between
function smooth_value = smooth_fall(val, hi, lo)
    if(hi>lo)
        disp('Error in smooth_fall(val, hi, lo): hi > lo');
        smooth_value = -1;
        return;
    end
    
    if(val>=lo)
        smooth_value = 0;
    elseif(val<hi)
        smooth_value = 1;
    else
        T = 2*(lo-hi);
        smooth_value = 0.5+0.5*sin(2*pi*(val-lo)/T - pi*0.5);
    end
end