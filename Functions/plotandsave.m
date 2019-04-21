function [] = plotandsave(origional, smoothed,mocap, path, filename, headless)
%PLOTANDSAVE Function to plot origional and smoothed data, then save

% Makes the folder to hold results
if ~exist(path, 'dir')
   mkdir(path)
end

[frames, ~] = size(origional); 
% Creates the x-axis coordinated
t = linspace(1,frames,frames);

figure;
% Hold allows both datasets to be scattered ontop the save figure
hold on
% Scatter raw data in red and Kalman output in blue and Mocap in green
scatter(t,origional,'r')
scatter(t,smoothed,'b')
% scatter(t,mocap,'g')
hold off
% Saves in respective ./Results folder
savefig(strcat(path,filename))

if headless
    % Closes figure for headless use
    close()
end

end

