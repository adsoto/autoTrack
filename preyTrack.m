function T = preyTrack(fpath,tVal,minBlob,radius)

% preyTrack runs a tracker on video stills. The program first prompts the
% user to select a region which contains predator and prey. This is used to
% create a background image that will be subtracted from all subsequent
% frames for image segmentation. The program finds the two largest blobs in
% the segmented binary image and selects the smaller one as the prey.
% Object area, orientation, and position are queried and stored. The
% program also has a visualizer for checking tracking progress. 

% TO DO: 
%   1) Save appropriate data to master data files (from previous codes)
%   2) incorporate this code into larger files for predator/eye tracking
%   3) include a more robust adaptive thresholding procedure
%   4) convert coordinates to local coordinates whereever necessary
 
close all

% set default parameters if not supplied by user
if nargin < 2
    tVal    = 70;
    minBlob = 25;
    radius  = 5;
    if nargin < 1
        % Check for Alberto's MacMini
        path = fullfile('Users','alberto','Documents','GitHub-SourceTree');

        if ~isempty(dir([filesep path]))
            % path to data 
            fpath = '/Volumes/VisualPred/ZF_visuomotor/Raw video/2016-02-19/S04';
        else
            % path to data on Alberto's MacBook
            fpath = '/Users/A_Soto/Desktop/S04';
        end
    end
end

% load filenames
a = dir([fpath filesep 'exp*']);

T.time         = 0;
T.frame_number = 0;
T.fps          = 250;
T.num_frames   = length(a);
T.currIm       = [];

% A custom visualizer for the Kalman state.
Visualizer.visualize = @visualize_prey;
Visualizer.paused    = false;
T.visualizer         = Visualizer;

while T.frame_number < T.num_frames
    T.frame_number = T.frame_number + 1;
    
    % current image filename
    T.currIm = a(T.frame_number).name;
    
    % load current frame
    frame = imread([fpath filesep T.currIm]);
    
    % background subtraction & threshold
    if T.frame_number == 1
        
        % Select region of interest around prey
        warning off
        imshow(frame,'InitialMagnification','fit');
        warning on
        title('Choose ROI around both fish');
        
        % Interactively find ROI
        h = impoly;
        roi_poly = wait(h);
        
        % Store results
        tmp = getPosition(h);
        roi.x = tmp(:,1);
        roi.y = tmp(:,2);
        
        delete(h), close all;
        
        % create a binary mask based on the selected ROI
        maskFish = roipoly(frame,roi.x,roi.y);
        
        % estimate background image
        imBkgnd = roifill(frame,maskFish);
        
        % store background image
        T.segmenter.background = imBkgnd;
    end
    
    % And threshold to get the foreground.
    T.segmenter.segmented = (abs(T.segmenter.background - frame)) > tVal;
    T.segmenter.segmented = imclose(T.segmenter.segmented, strel('disk', radius));
    
    % find connected components in binary (segmented) image
    cc = bwconncomp(T.segmenter.segmented);
    
    % measure region properties of connected components
    ccProps = regionprops(cc,'Area','Orientation','Centroid');
    
    % find the largest blobs
    idx = find([ccProps.Area] > minBlob);
    
    % keep only the larger connected components, i.e., fish blobs
    ccProps = ccProps(idx);
    
    % define binary mask of fish blobs
    T.recognizer.fishBlobs = ismember(labelmatrix(cc), idx);
    
    % Make sure at lease one blob was recognized
    if ~isempty(T.recognizer.fishBlobs)
        
        % take the smaller (prey) blob
        % TO DO: Think of a way to use the predator blob for something useful
        [I, IX] = min([ccProps.Area]);
        
        T.preySize = I;
        
        % save the prey's position and orientation
        T.prey.orientation = ccProps(IX(size(IX,2))).Orientation;
        T.prey.position = ccProps(IX(size(IX,2))).Centroid;
    end
    
    % visualize progress of tracking
    if isfield(T, 'visualizer')
        T = T.visualizer.visualize(T, frame);
    end
    
    % update time vector
    T.time = T.time + 1/T.fps;
    
end

function T = visualize_prey(T, frame)
% VISUALIZE_KALMAN - a visualizer for a Kalman filter tracker.
%
% NOTE: This function is intended to be run as a VISUALIZER in the
% tracking framework.  See documentation for RUN_TRACKER.
%
% VISUALIZE_KALMAN(T, frame) displays the current image in 'frame',
% along with the measurement and current tracker estimate in a
% figure window.
%
% See also: kalman_tracker, run_tracker.

% Initialize the figure and setup pause callback.
if ~isfield(T.visualizer, 'init');
  figure;
  h = gcf;
  set(h, 'KeyPressFcn', {@pauseHandler, h});
  setappdata(h, 'paused', false);
  T.visualizer.init = true;
end

% display the current fish blobs (as dark blobs)
% figure, 
ax1 = subplot(2,2,1);
imshow(imcomplement(T.segmenter.segmented),'InitialMagnification','fit')
ax1.Title.String = 'Segmented Image';

ax2 = subplot(2,2,2);
imshow(imcomplement(T.recognizer.fishBlobs),'InitialMagnification','fit')
ax2.Title.String = 'Fish Blobs';

% % display the current fish blobs
% subplot(1,2,1)
% imshow(T.recognizer.fishBlobs,'InitialMagnification','fit')
    
% Display the current frame.
ax3 = subplot(2,2,[3 4]);
imshow(frame,'InitialMagnification','fit');
hold on;
ax3.Title.String = 'Tracking Status';

% draw the current position
plot(ax3, T.prey.position(1),T.prey.position(2),'ro','MarkerSize',10)
hold off;

% % Draw the current measurement in red.
% if isfield(T.representer, 'BoundingBox')
%   rectangle('Position', T.representer.BoundingBox, 'EdgeColor', 'r');
% end
% 
% % And the current prediction in green
% if isfield(T.tracker, 'm_k1k1')
%   rectangle('Position', T.tracker.m_k1k1, 'EdgeColor', 'g');
% end
drawnow;

% If we're paused, wait (but draw).
while (getappdata(gcf, 'paused'))
  drawnow;
end
return

% This is a callback function that toggles the pause state.
function pauseHandler(a, b, h)
setappdata(h, 'paused', xor(getappdata(h, 'paused'), true)) ;
return
