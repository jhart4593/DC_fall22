CustomInput = 0; % specify custom input and run single instance
RecreateReportResults = 1; % Recreate report results (note this will take a few minutes)


if CustomInput
    clc,clear all

    Density = [1026 1026]; 
    traj = 1; 
    CurrBetaSel = 1; 
    CurrVelSel = 1; 
    TsSelection = 1;
    PlotProfiles = 1; 
    run('SimulateRemusEqnOfMotion.m')
end
if RecreateReportResults
    
    % Sampling Time Study
    % Case 1: Ideal sampling time
    clc,clear all
    Density = [1026, 1026]; 
    traj = 1; 
    CurrBetaSel = 1; 
    CurrVelSel = 1; 
    TsSelection = 1;
    PlotProfiles = 1;
    run('SimulateRemusEqnOfMotion.m')

    % Case 2: Too fast
    clc,clear all
    Density = [1026, 1026]; 
    traj = 1; 
    CurrBetaSel = 1; 
    CurrVelSel = 1; 
    TsSelection = 2;
    PlotProfiles = 0; 
    run('SimulateRemusEqnOfMotion.m')
    
    % Trajectory Tracking/Disturbance robustness

    % Case 1: Trajectory 1, fast speed disturbances 
    clc,clear all
    Density = [1026, 1026]; 
    traj = 1; 
    CurrBetaSel = 1; 
    CurrVelSel = 2; 
    TsSelection = 1;
    PlotProfiles = 0; 
    run('SimulateRemusEqnOfMotion.m')
    
    % Case 2: Trajectory 2, fast speed disturbances 
    clc,clear all
    Density = [1026, 1026]; 
    traj = 2; 
    CurrBetaSel = 1; 
    CurrVelSel = 2; 
    TsSelection = 1;
    PlotProfiles = 0; 
    run('SimulateRemusEqnOfMotion.m')
    
    % Case 3: Trajectory 3, fast speed disturbances 
    clc,clear all
    Density = [1026, 1026]; 
    traj = 3; 
    CurrBetaSel = 1; 
    CurrVelSel = 2; 
    TsSelection = 1;
    PlotProfiles = 0; 
    run('SimulateRemusEqnOfMotion.m')
    
    % Case 4: Nonuniform density
    clc,clear all
    Density = [800, 1026]; 
    traj = 1; 
    CurrBetaSel = 1; 
    CurrVelSel = 1; 
    TsSelection = 1;
    PlotProfiles = 0; 
    run('SimulateRemusEqnOfMotion.m')

end

