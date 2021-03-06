% Test ESDS on the LASA HandWritten dataset.
% 
% Author: Matteo Saveriano
%
% For further details, see:
% @article{Saveriano2021Energy,
%   title="An Energy-based Approach to Ensure the Stability of Learned
% Dynamical Systems",
%   author="M. Saveriano",
%   journal="IEEE International Conference on Robotics and Automation",
%   pages="4407--4413"
%   year="2020"
% }
%
% Copyright (c) 2021 Matteo Saveriano, Dept. of Computer Science and 
% Digital Science Center, University of Innsbruck, 6020, Innsbruck,
% Austria, https://iis.uibk.ac.at
%

close all
clear

addpath(genpath('LASA_dataset'));
addpath(genpath('lib'));

% Parameters
spaceScale = 100; % Data in the dataset are in 'cm'. Use this to scale differently
demoNum = [1 2 3]; % Use only first 3 demos
samplingRate = 10; % Resample to speed-up

simOptions.s          = spaceScale;
% simOptions.a -> Coefficient of the class K function.
% simOptions.a = 0.1 or 0.01 work well with data in 'cm', i.e. spaceScale=1
simOptions.a          = 0.01; % 
simOptions.iterNum    = 1000;
simOptions.tol        = spaceScale * 0.1; % 1 mm
simOptions.dsOrder    = 1; 
simOptions.goal       = [0; 0];
simOptions.plotResult = 0;

PLOT_ON = 1; % 1 - plot generated trajectory.

%% Learning loop
esdsTrainingTime = zeros(1,26);
for modIt=1:26
    %% Load LASA motion and resample
    [demos, dt]   = load_LASA_models('', modIt);
    simOptions.dt = samplingRate*dt;
    demoSize = size(demos{1}.pos, 2);
    sInd = [1:samplingRate:demoSize, demoSize];
    
    %% Create training set for ESDS
    xDemo  = [];
    xdDemo = [];
   
    timeData = 0;
    totEnergy = zeros(1,length(demoNum));
    for demoIt=demoNum   
        %% Scale demonstration
        demos{demoIt}.pos = spaceScale * demos{demoIt}.pos;
        demos{demoIt}.vel = spaceScale * demos{demoIt}.vel;
    
        xCurr = demos{demoIt}.pos(:,sInd);
        vCurr = [diff(xCurr,[],2)./simOptions.dt [0;0]];
        
        xDemo  = [xDemo xCurr];
        xdDemo = [xdDemo vCurr];
        
        x0(:,demoIt) = demos{demoIt}.pos(:,1);
        
        %% Compute variable 'a' gain to have u = 0.1 * spaceScale after 'end-P' steps
        %  Note: This is used to consider data in different units (cm, mm, ...)
        %        and it is not contained in the original paper where we set
        %        simOptions.a = 0.01.
        P = 5;
        xx = xCurr(:,end-P);
        dxx = xdDemo(:,end-P);
        a(demoIt) = (0.1 * spaceScale) / (norm(dxx+xx) * dot(xx,xx));
                
        %% Estimate dissipated energy in worst case (alpha = 0 && beta = 1)
        tic;
        vTildeD = vCurr + xCurr;
        sDotDemo = -simOptions.dt.*dot(xCurr,vTildeD);
        totEnergy(demoIt) = sum(sDotDemo(sDotDemo<0));
        timeData = timeData + toc;
    end
    simOptions.a = max(a); % Comment this to use a constant 'a'
    
    %% Define maximum tank energy
    simOptions.sMax = max(abs(totEnergy));
    
    %% Scale training velocity using a class K function
    tic;
    alphaInv = 1./1-exp(-simOptions.a*((dot(xDemo,xDemo))));
    alphaInv(isinf(alphaInv)) = 1;   
    trainingData = [xDemo; (xdDemo+xDemo).*repmat(alphaInv,2,1)];
    timeData = timeData + toc;

    tmpTraining = [];
    for nbStates = 4:7 % Find best number of components
        %% Train GMM/GMR
        tic;
        [Priors, Mu, Sigma] = EM_init_kmeans(trainingData, nbStates);
        [Priors, Mu, Sigma] = EM(trainingData, Priors, Mu, Sigma);
        tmpTraining = [tmpTraining toc];

        DSHandle = @(x) GMR(Priors, Mu, Sigma, x, 1:2,3:4);

        %% Swept Area and Velocty rmse distances
        [pos, vel] = simulate_ESDS(DSHandle, x0, simOptions);
        for demoIt=1:length(pos)
            tmpSEA(nbStates-3, demoIt) = swept_area_error(pos{demoIt}, demos{demoIt}.pos(:,sInd));   
            demoX = demos{demoIt}.pos(:,sInd);
            demoV = [diff(demoX,[],2)./simOptions.dt [0;0]];
            tmpRMSE(nbStates-3, demoIt) = velocity_rmse(DSHandle, demoX, demoV, simOptions.dt, simOptions.sMax);
        end
        
        %% Plot generated trajectories
        if(PLOT_ON)
            scatter(xDemo(1,:),xDemo(2,:),[],[217,83,25]./255)
            hold on;
            for l=1:length(pos)
                plot(pos{l}(1,:),pos{l}(2,:),'k','LineWidth',2)
                plot(pos{l}(1,end),pos{l}(2,end),'k.','markersize',40)
            end
            box on;
            pause(1);
            close all;
        end
    end
    %% Store only best results
    meanSEA = mean(tmpSEA,2);
    [val, ind] = min(meanSEA);
    allSEA(modIt,:) = tmpSEA(ind, :);
    allVRMSE(modIt,:) = tmpRMSE(ind, :);
    esdsTrainingTime(modIt) = tmpTraining(ind) + timeData;
end

%% Get global results
disp(['Mean distance: ' num2str(mean(mean(allSEA,2)))]);
disp(['Max distance: ' num2str(max(mean(allSEA,2)))]);
