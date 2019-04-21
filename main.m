% Load	dataset
load	./Data/Subject7-Session3-Take1_alljoints_matched.mat
% Clear results from old runs
if exist('./Results', 'dir')
   rmdir('./Results','s');
end
% True saves figures and closes, false leaves them open
headless = true;

bodyDetectors = cat(5, body2D, coco2D, msra2D);
bodyDetectorNames = ["body2D"; "coco2D"; "msra2D"];
motionModels = ["Constant_Velocity"; "Constant_Acceleration"];
% Loop through the 3 given datasets
for detector = 3:3
    % Loop through both views of the dataset
    for view = 1:1
        % Loop through each of the 12 body joints
        for joint = 1:1
            for model = 1:1
               motionModel = motionModels(model);
               
               % Extract joint data, now in the form of a n x 3 matrix
                points = squeeze(bodyDetectors(view, :, joint,:, detector)); 
                % Smooth using a Kalman filter
                smoothedPoints = kalman(points, motionModel);

                % Plot the origional and Kalman smoothed data           
                x = squeeze(points(:,1));
                y = squeeze(points(:,2));

                xMocap = squeeze(mocap2D(view,:,joint,1));
                yMocap = squeeze(mocap2D(view,:,joint,2));

                xSmoothed = squeeze(smoothedPoints(:,1));
                ySmoothed = squeeze(smoothedPoints(:,2));   

                % Path to save figure
                % Ex: './Results/coco2D/view_1/joint_1/'
                path = strcat('./Results/',motionModel,'/First_Pass/');
                path = strcat(path, bodyDetectorNames(detector),'/view_',num2str(view),'/joint_',num2str(joint),'/');
                % Plot the x coordinate origional and smoothed data
                plotandsave(x, xSmoothed, xMocap, path, 'x', headless)
                % Plot the x coordinate origional and smoothed data
                plotandsave(y, ySmoothed, yMocap, path, 'y', headless)

                secondSmoothedPoints = flip(kalman(flip(smoothedPoints), motionModel));

                xSecondSmoothed = squeeze(secondSmoothedPoints(:,1));
                ySecondSmoothed = squeeze(secondSmoothedPoints(:,2));

                % Path to save figure
                % Ex: './Results/coco2D/view_1/joint_1/'
                path = strcat('./Results/',motionModel,'/Second_Pass/');
                path = strcat(path, bodyDetectorNames(detector),'/view_',num2str(view),'/joint_',num2str(joint),'/');
                % Plot the x coordinate origional and smoothed data
                plotandsave(xSmoothed, xSecondSmoothed, xMocap, path, 'x', headless)
                % Plot the x coordinate origional and smoothed data
                plotandsave(ySmoothed, ySecondSmoothed, yMocap, path, 'y', headless)
            end
            
        end
       
    end
    
end
