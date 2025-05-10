% Inverted Pendulum Simulation with and without Heel Support
clear; clc; close all;  % Clear workspace, clear command window, and close all figures

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                                                                     %%%
%%% > Simulating Ankle-Joint Misalignment effects with AFOs during -    %%%
%%%   - Stepping Up Motion                                              %%%
%%%   - Stepping Down Motion                                            %%%
%%%   - Preferred Walking                                               %%%
%%% > Authors : K.Badari Vishal; Ganesh M. Bapat                        %%%
%%%               ---------------------------------------               %%%
%%% > Data taken from "An Extensive Set of Kinematic & Kinetic Data for %%%
%%%   Individuals with Intact Limbs & Transfemoral Prosthesis Users".   %%%
%%%   DOI : https://doi.org/10.1155/2020/8864854                        %%%
%%%               ---------------------------------------               %%%
%%% > MATLAB Code                                                       %%%
%%%   - Edited by : K.Badari Vishal, 3rd April 2022                     %%%
%%%   - Last Edited by : K.Badari Vishal, 11th August 2022              %%%
%%%               ---------------------------------------               %%%
%%% > Analysis Details                                                  %%%
%%%   - All Lengths & Distances are taken in "mm".                      %%%
%%%   - All Angles are taken in "Degrees".                              %%%
%%%   - Theta in Counter Clock-wise(CCW) direction is considered "+ve". %%%
%%%   - Theta in Clock-wise(CW) direction is "-ve".                     %%%
%%%   - Conventions:                                                    %%%
%%%      ~ Anterior (->)  : +ve                                         %%%
%%%      ~ Posterior (<-) : -ve                                         %%%
%%%      ~ Proximal (^)   : +ve                                         %%%
%%%      ~ Distal (v)     : -ve                                         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;
close all;

addpath('Data')
addpath('Functions')

OCS_Radius = 15; % Circular range for OCS misalignments (-15mm to 15mm)

% Selecting Motion Type
disp('Initializing...')
disp('-')
motion = motion_type();

% If motion type is not selected, end the program.
if motion == 0
    disp('Motion Type is not selected.')
    disp('Program terminated.')
    return
else
    disp('Motion Input complete.')
    disp('-')
end

% Loading respective Datasets
disp('Loading Datasets...')
disp('-')
[apmDataset,limbAnglesDataset,timeStamps,index] = load_Dataset(motion);
if isequal(apmDataset,0)
    disp('No Dataset selected!')
    disp('Program terminated.')
    return
else
    disp('Datasets Loaded.')
    disp('-')
end

% Choosing input type for assigning misalignments (Graphical OR Typical)
disp('Choosing Input Type...')
disp('-')
input_type = questdlg('Choose type of input for misalignments:', ...
    'Misalignments Input Type', ...
    'Typical Input','Graphical Input','Typical Input');
switch input_type
    case 'Typical Input'
        [deltaX,deltaY] = typeInput_misalignments(OCS_Radius);
        Mis_Type = misalignmentType(deltaX,deltaY);
    case 'Graphical Input'
        [deltaX,deltaY] = graphInput_misalignments(apmDataset,OCS_Radius);
        Mis_Type = misalignmentType(deltaX,deltaY);
end

% Checking if misalignments are given:
if isempty(deltaX) || isempty(deltaY)
    return
else
    disp('Misalignmentes set.')
    disp('-')
end

% Analysis & Computation
disp('Performing Analysis...')
disp('-')
% LLD - Left Limb Data; RLD = Right Limb data
LLD = afoMotionAnalysis(apmDataset,limbAnglesDataset,timeStamps,index,deltaX,deltaY,'Left',motion);
RLD = afoMotionAnalysis(apmDataset,limbAnglesDataset,timeStamps,index,deltaX,deltaY,'Right',motion);
disp('Analysis Complete.')

% Plotting
% Plot-1 : Individual
relativeMotionPlot1(LLD,Mis_Type,motion)
relativeMotionPlot1(RLD,Mis_Type,motion)
% Plot-2 : Combined
relativeMotionPlot2(LLD,RLD,Mis_Type,motion)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%---------------------------------------------------------------------%%%
%%%                          DEFINED FUNCTIONS                          %%%
%%%---------------------------------------------------------------------%%%

%-------------------------------------------------------------------------%
% Function for user input of Motion type

function motion = motion_type()
    motion_list = {'Preferred Walking';'Stepping Up';'Stepping Down'};
    [index,tf] = listdlg('PromptString',{['Following is the table ' ...
        'showing different'], ...
        'motion types considered in this analysis.', ...
        'Select any motion type for further analysis:'}, ...
        'SelectionMode','single','ListString',motion_list, ...
        'ListSize',[250,80],'Name','Motion Type Selection');
    if tf == 1
        motion = index;
    elseif tf == 0
        motion = 0;
    end
end

%-------------------------------------------------------------------------%
% Function for loading Data

function [apmDataset,limbAnglesDataset,timeStamps,index] = load_Dataset(motion)
    % apmDataset : Anthropometric Dataset
    % limbAnglesDataset : Shank and Foot angles dataset
    % timeStamps : Time limits for each dataset

    % Data Sets used
    data_sets = {'PreferredWalking.xlsx';'SteppingUp.xlsx';
        'SteppingDown.xlsx'};

    % Loading datasets for respective motion and participant
    if isequal(motion,1) % Preferred Walking

        % Dataset names & ranges
        sheet_names = {'AB01_Trial_1';'AB01_Trial_2';
            'AB02_Trial_1';'AB02_Trial_2';
            'AB03_Trial_1';'AB03_Trial_2';
            'AB04_Trial_1';'AB04_Trial_2';
            'AB05_Trial_1';'AB05_Trial_2'};
        data_range = {'AX2:BA3101';'AX2:BA3101';
            'AX2:BA3101';'AX2:BA3101';
            'AX2:BA1351';'AX2:BA1337';
            'AX2:BA2378';'AX2:BA2378';
            'AX2:BA2465';'AX2:BA2656'};
        timeStamps_range = {'A2:A3101';'A2:A3101';
            'A2:A3101';'A2:A3101';
            'A2:A1351';'A2:A1337';
            'A2:A2378';'A2:A2378';
            'A2:A2465';'A2:A2656'};

        % Choosing respective Motion Trial
        [index,tf] = listdlg('PromptString',{'Choose any trial dataset for', ...
            'the given motion type:'},'SelectionMode','single', ...
            'ListString',sheet_names, ...
            'ListSize',[250,250],'Name','Motion Trial Selection');
        apm = cell2mat(sheet_names(index));
        if tf == 0
            apmDataset = 0;
            limbAnglesDataset = 0;
            timeStamps = 0;
        elseif tf == 1
            apmDataset = readcell(cell2mat(data_sets(motion)), ...
                'Sheet',apm(1:4),'Range','A8:G19');
            limbAnglesDataset = readcell(cell2mat(data_sets(motion)), ...
                'Sheet',char(sheet_names(index)), ...
                'Range',char(data_range(index)));
            timeStamps = readcell(cell2mat(data_sets(motion)), ...
                'Sheet',char(sheet_names(index)), ...
                'Range',char(timeStamps_range(index)));
        end

    elseif isequal(motion,2) % Stepping Up

        % Dataset names & ranges
        sheet_names = {'AB01_Trial_1';'AB01_Trial_2';
            'AB02_Trial_1';'AB02_Trial_2';
            'AB03_Trial_1';'AB03_Trial_2';
            'AB04_Trial_1';'AB04_Trial_2';
            'AB05_Trial_1';'AB05_Trial_2'};
        data_range = {'AX2:BA562';'AX2:BA585';
            'AX2:BA514';'AX2:BA819';
            'AX2:BA606';'AX2:BA509';
            'AX2:BA524';'AX2:BA636';
            'AX2:BA494';'AX2:BA590'};
        timeStamps_range = {'A2:A562';'A2:A585';
            'A2:A514';'A2:A819';
            'A2:A606';'A2:A509';
            'A2:A524';'A2:A636';
            'A2:A494';'A2:A590'};

        % Choosing respective Motion Trial
        [index,tf] = listdlg('PromptString',{'Choose any trial dataset for', ...
            'the given motion type:'},'SelectionMode','single', ...
            'ListString',sheet_names, ...
            'ListSize',[250,250],'Name','Motion Trial Selection');
        apm = cell2mat(sheet_names(index));
        if tf == 0
            apmDataset = 0;
            limbAnglesDataset = 0;
            timeStamps = 0;
        elseif tf == 1
            apmDataset = readcell(cell2mat(data_sets(motion)), ...
                'Sheet',apm(1:4),'Range','A8:G19');
            limbAnglesDataset = readcell(cell2mat(data_sets(motion)), ...
                'Sheet',char(sheet_names(index)), ...
                'Range',char(data_range(index)));
            timeStamps = readcell(cell2mat(data_sets(motion)), ...
                'Sheet',char(sheet_names(index)), ...
                'Range',char(timeStamps_range(index)));
        end

    elseif isequal(motion,3) % Stepping Down

        % Dataset names & ranges
        sheet_names = {'AB01_Trial_1';'AB01_Trial_2';
            'AB02_Trial_1';'AB02_Trial_2';
            'AB03_Trial_1';'AB03_Trial_2';
            'AB04_Trial_1';'AB04_Trial_2';
            'AB05_Trial_1';'AB05_Trial_2'};
        data_range = {'AX2:BA549';'AX2:BA489';
            'AX2:BA507';'AX2:BA536';
            'AX2:BA508';'AX2:BA479';
            'AX2:BA525';'AX2:BA522';
            'AX2:BA635';'AX2:BA529'};
        timeStamps_range = {'A2:A549';'A2:A489';
            'A2:A507';'A2:A536';
            'A2:A508';'A2:A479';
            'A2:A525';'A2:A522';
            'A2:A635';'A2:A529'};

        % Choosing respective Motion Trial
        [index,tf] = listdlg('PromptString',{'Choose any trial dataset for', ...
            'the given motion type:'},'SelectionMode','single', ...
            'ListString',sheet_names, ...
            'ListSize',[250,250],'Name','Motion Trial Selection');
        apm = cell2mat(sheet_names(index));
        if tf == 0
            apmDataset = 0;
            limbAnglesDataset = 0;
            timeStamps = 0;
        elseif tf == 1
            apmDataset = readcell(cell2mat(data_sets(motion)), ...
                'Sheet',apm(1:4),'Range','A8:G19');
            limbAnglesDataset = readcell(cell2mat(data_sets(motion)), ...
                'Sheet',char(sheet_names(index)), ...
                'Range',char(data_range(index)));
            timeStamps = readcell(cell2mat(data_sets(motion)), ...
                'Sheet',char(sheet_names(index)), ...
                'Range',char(timeStamps_range(index)));
        end
    else
        apmDataset = 0;
        limbAnglesDataset = 0;
        timeStamps = 0;
    end
end

%-------------------------------------------------------------------------%
% Function for taking typical input for misalignments

function [dX,dY] = typeInput_misalignments(OCS_Radius)
    prompt = {'Enter misalignment in x-direction (deltaX) (-15<0<15mm):';
        'Enter misalignnment in y-direction (deltaY) (-15<0<15mm):'};
    dlgtitle = 'Misalignment Input';
    dims = [1 70];
    definput = {'0','0'};
    deltas = inputdlg(prompt,dlgtitle,dims,definput);
    if isempty(deltas)
        dX = '';
        dY = '';
        disp('No Misalignments were given.')
        disp('Program Terminated.')
        return
    else
        [dX,dY] = ocsPoints(str2double(cell2mat(deltas(1))), ...
            str2double(cell2mat(deltas(2))),OCS_Radius);
    end
end

%-------------------------------------------------------------------------%

% Function for taking graphical input for misalignments

function [dX,dY] = graphInput_misalignments(apm_dataset,OCS_Radius)
    ACS = cell2mat(apm_dataset(1,2:4));
    FHEE = cell2mat(apm_dataset(2,2:4));
    FMT5 = cell2mat(apm_dataset(3,2:4));
    KNEE = cell2mat(apm_dataset(4,2:4));    

    msgbox({'- Given circle in red is a constrained selection range for ' ...
        'Orthotic Coordinate System (OCS) center.' ...
    '','- Click a point on/in the circle to select OCS center accordingly.' ...
    '',['- If clicked out of range, the input auto-corrects itself & ' ...
    'selects the outermost point on the circle ' ...
    'at the same angle of clicked input.']},'Input','help','modal');
    inpFig = figure('Name','GInput Figure','NumberTitle','off');
    plot([ACS(1),FHEE(1)],[ACS(2),FHEE(2)],'k-','MarkerSize',2)
    hold on;
    plot([ACS(1),FMT5(1)],[ACS(2),FMT5(2)],'k-','MarkerSize',2)
    plot(ACS(1),ACS(2),'go','MarkerSize',5)
    plot([FMT5(1),FHEE(1)],[FMT5(2),FHEE(2)],'k-','MarkerSize',2)
    plot(KNEE(1),KNEE(2),'ko','MarkerSize',5)
    plot([ACS(1),KNEE(1)],[ACS(2),KNEE(2)],'k-','MarkerSize',2)
    arc(0,0,OCS_Radius,2*pi) % OCS Misalignment allotment range
    xlim([-250,250]),ylim([-90,500])
    xticks([]),yticks([])    
    [X,Y] = ginput(1);
    [dX,dY] = ocsPoints(X,Y,OCS_Radius);
    plot(dX,dY,'bo','MarkerSize',5)
    hold off;
    legend('Limb','','ACS','','Knee Joint','','','OCS','Location','bestoutside')

    q_ans = questdlg('Keep the figure open or Clost it?', ...
        'GInput Figure Status', ...
        'Keep','Close','Keep');
    switch q_ans
        case 'Keep'
            return
        case 'Close'
            close(inpFig);
    end    
end

%-------------------------------------------------------------------------%

% Arc / Circle
function arc(x,y,r,theta)
    hold on;
    th = -theta+(pi/2):pi/50:theta+(pi/2);
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    plot(xunit, yunit,'r-','MarkerSize',2)
end

%-------------------------------------------------------------------------%

% OCS Points
function [deltaX,deltaY] = ocsPoints(X,Y,OCS_Radius)
    R = OCS_Radius; % Radius of OCS Circle
    R_ = sqrt(X^2 + Y^2);
    th = abs(atand(Y/X));
    if R_<=R
        deltaX = X;
        deltaY = Y;
    elseif R_>R
        if X>=0 && Y>=0
            deltaX = R*cosd(th);
            deltaY = R*sind(th);
        elseif X>=0 && Y<=0
            deltaX = R*cosd(th);
            deltaY = -R*sind(th);
        elseif X<=0 && Y<=0
            deltaX = -R*cosd(th);
            deltaY = -R*sind(th);
        elseif X<=0 && Y>=0
            deltaX = -R*cosd(th);
            deltaY = R*sind(th);
        end
    end
end

%-------------------------------------------------------------------------%

% Function for determining Misalignment Type
function Mis_Type = misalignmentType(deltaX,deltaY)
    
    % Misalignment type with reference to deltaX
    if deltaX>0
        X_Misalignment = 'Anterior';
    elseif deltaX<0
        X_Misalignment = 'Posterior';
    else
        X_Misalignment = '';
    end
    
    % Misalignment type with reference to deltaY
    if deltaY>0
        Y_Misalignment = 'Proximal';
    elseif deltaY<0
        Y_Misalignment = 'Distal';
    else
        Y_Misalignment = '';
    end
    
    % Misalignment type with reference to deltaX & deltaY
    if strlength(X_Misalignment)==0 && strlength(Y_Misalignment)==0
        Mis_Type = 'None';
    else
        Mis_Type = [X_Misalignment,'-',Y_Misalignment];
    end
end
%-------------------------------------------------------------------------%% --- Fixed Parameters ---
mass = 1.0;                   % Mass of the body segment (kg)
segment_length = 0.5;         % Length of the body segment (m)
g = 9.81;                     % Acceleration due to gravity (m/s^2)
theta0 = deg2rad(10);         % Initial angle (converted from 10 degrees to radians)
omega0 = 0;                   % Initial angular velocity (rad/s)
T_end = 10;                   % Duration of the simulation (10 seconds)
heel_effect_factor = 0.9;     % Heel support reduces effective length by 10%
fps = 60;                     % Frames per second for the animation

% Effective lengths for with and without heel support
effective_lengths = [heel_effect_factor * segment_length, segment_length]; % [With Heel, Without Heel]

% Initial conditions for the simulation (initial angle and angular velocity)
initial_conditions = [theta0; omega0];

% Time span for the ode45 solver (from 0 to T_end)
time_span = [0, T_end];

% --- Solve ODEs ---
solutions = cell(2, 1);  % Cell array to store solutions for both conditions (with and without heel)
times = cell(2, 1);      % Cell array to store time vectors for both conditions

% Solve the ODE for both cases (with and without heel)
for i = 1:2
    % Define the ODE function for the pendulum with respect to its angular position and velocity
    pendulum_ode = @(t, y) [y(2); -(g / effective_lengths(i)) * sin(y(1))];
    
    % Solve the ODE using ode45, which returns the time vector and solution matrix (theta, omega)
    [times{i}, solutions{i}] = ode45(pendulum_ode, time_span, initial_conditions);
end

% Extract angles (theta) and angular velocities (omega) from the solution matrix
theta_heel = solutions{1}(:, 1);  % Angle with heel support
omega_heel = solutions{1}(:, 2);  % Angular velocity with heel support
theta_no_heel = solutions{2}(:, 1);  % Angle without heel support
omega_no_heel = solutions{2}(:, 2);  % Angular velocity without heel support

% --- Plot Results ---
figure;  % Create a new figure for plotting

% Plot angle (in degrees) over time
subplot(2, 1, 1);  % Create the first subplot (upper plot)
plot(times{1}, rad2deg(theta_heel), 'b', 'LineWidth', 1.5); hold on;  % Plot the angle with heel support in blue
plot(times{2}, rad2deg(theta_no_heel), 'r', 'LineWidth', 1.5);  % Plot the angle without heel support in red
title('Inverted Pendulum - Angle Over Time');  % Title of the plot
xlabel('Time (s)');  % X-axis label
ylabel('Angle (degrees)');  % Y-axis label
legend('With Heel Support', 'Without Heel Support');  % Legend for the plot
grid on;  % Turn on the grid

% Plot angular velocity (in rad/s) over time
subplot(2, 1, 2);  % Create the second subplot (lower plot)
plot(times{1}, omega_heel, 'b', 'LineWidth', 1.5); hold on;  % Plot angular velocity with heel support in blue
plot(times{2}, omega_no_heel, 'r', 'LineWidth', 1.5);  % Plot angular velocity without heel support in red
title('Inverted Pendulum - Angular Velocity Over Time');  % Title of the plot
xlabel('Time (s)');  % X-axis label
ylabel('Angular Velocity (rad/s)');  % Y-axis label
legend('With Heel Support', 'Without Heel Support');  % Legend for the plot
grid on;  % Turn on the grid

disp('ODE45 Simulation completed.');  % Display a message when the ODE simulation is done

% --- Real-Time Animation ---
figure;  % Create a new figure for animation
animation_length = 1.2 * segment_length;  % Set the axis limit for animation (slightly larger than segment length)

colors = ['b', 'r'];  % Define colors for the two cases (with heel in blue, without heel in red)

% Set up subplots for animation (one subplot for each condition)
for i = 1:2
    subplot(1, 2, i);  % Create two subplots in a single row (1 row, 2 columns)
    hold on;  % Hold the plot to update it in each frame
    if i == 1
        title('With Heel Support');  % Set title for the first subplot
    else
        title('Without Heel Support');  % Set title for the second subplot
    end
    xlabel('X Position (m)');  % X-axis label
    ylabel('Y Position (m)');  % Y-axis label
    axis equal;  % Set equal scaling for both axes
    axis([-animation_length animation_length -animation_length animation_length]);  % Set axis limits
    grid on;  % Turn on the grid
end

% Precompute interpolated solutions for animation (to match the frame rate)
t_anim = 0:(1 / fps):T_end;  % Time vector for animation (with intervals based on FPS)
theta_interp = cell(2, 1);  % Cell array to store interpolated angles for both cases

% Interpolate the angle solutions for both cases (heel support and no heel)
for i = 1:2
    theta_interp{i} = interp1(times{i}, solutions{i}(:, 1), t_anim, 'linear', 'extrap');  % Linear interpolation with extrapolation
end

% Initialize animation objects (lines and markers for each pendulum)
lines = gobjects(2, 1);  % Initialize graphics objects for the lines (pendulum rods)
markers = gobjects(2, 1);  % Initialize graphics objects for the markers (pendulum bob)

% Set up the line and marker for both subplots (with and without heel support)
for i = 1:2
    subplot(1, 2, i);  % Select the correct subplot
    lines(i) = line([0, 0], [0, 0], 'Color', colors(i), 'LineWidth', 2);  % Create line for pendulum rod
    markers(i) = plot(0, 0, 'o', 'Color', colors(i), 'MarkerSize', 10, 'MarkerFaceColor', colors(i));  % Create marker for pendulum bob
end

% Animation loop
disp('Starting real-time animation...');  % Display message when animation starts
for i = 1:length(t_anim)  % Loop over each time step in the animation
    for j = 1:2  % Loop over the two cases (heel support and no heel support)
        % Calculate the x and y positions of the pendulum bob
        x = effective_lengths(j) * sin(theta_interp{j}(i));  % X position
        y = -effective_lengths(j) * cos(theta_interp{j}(i));  % Y position
        
        % Update the line (pendulum rod) and marker (bob) positions
        set(lines(j), 'XData', [0, x], 'YData', [0, y]);  % Update the rod line
        set(markers(j), 'XData', x, 'YData', y);  % Update the bob position
    end
    pause(1 / fps);  % Pause for the frame rate duration
end

disp('Real-time animation completed.');  % Display message when animation is done