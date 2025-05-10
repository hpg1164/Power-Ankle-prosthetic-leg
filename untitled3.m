% Load the data from the files
gait_data = load('data_2.txt'); % Load gait data

% Create time vectors for each dataset
time_gait = linspace(0, 1.5, length(gait_data)); % Time vector for gait data

% Plot the dataset
figure;
plot(time_gait, gait_data, 'b-', 'LineWidth', 2); % Plot gait data (blue solid line)
hold on;

% Add labels and title
title('Gait path of ankle','FontSize',20);
xlabel('Time[s]','FontSize',20);
ylabel('Angle (Degree)','FontSize',20);
grid on;

% Initialize the circle
h = plot(time_gait(1), gait_data(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% Prepare to save as GIF
gifFile = 'gait_animation_1.gif'; % Name of the output GIF file
delayTime = 0.05; % Delay between frames (controls speed of the animation)

% Animate the circle along the line and capture frames
for i = 1:length(time_gait)
    % Update the position of the circle
    set(h, 'XData', time_gait(i), 'YData', gait_data(i));
    
    % Capture the current frame
    frame = getframe(gcf); % Capture the entire figure window
    im = frame2im(frame); % Convert the frame to an image
    [imind, cm] = rgb2ind(im, 256); % Convert the image to indexed format
    
    % Write the frame to the GIF file
    if i == 1
        imwrite(imind, cm, gifFile, 'gif', 'Loopcount', inf, 'DelayTime', delayTime);
    else
        imwrite(imind, cm, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
    end
    
    % Pause to create the animation effect
    pause(delayTime);
end

hold off;