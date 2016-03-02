function T = autoTrack(fpath, gamma, tau, radius)
% autoTrack - is a zero-order Kalman filter tracker.
%
% autoTrack(fpath, gamma, tau, radius) runs a tracker on video stills
% located in fpath, with background subtraction parameters gamma, tau
% and radius. All necessary subroutines are contained within this file.
% Original source code, from which this was adapted, was written by 
% Andrew D. Bagdanov, PhD; http://www.micc.unifi.it/bagdanov/tracking
%
% Inputs:
%  fpath   - path to video stills to process.
%  gamma   - gamma parameter for background subtraction.
%  tau     - tau parameter for background subtraction.
%  radius  - radius for closing in background subtraction.


% set default parameters if not supplied by user
if nargin < 2
    gamma   = 0.05;
    tau     = 50;
    radius  = 3;
    if nargin < 1
        fpath = '/Volumes/VisualPred/ZF_visuomotor/Raw video/2016-02-18/S01';
    end
end

%% Initialize modules 

% NOTE: Tracker module may change depending on system & measurement models 

% Initialize background model parameters
Segmenter.gamma   = gamma;
Segmenter.tau     = tau;
Segmenter.radius  = radius;
Segmenter.segment = @background_subtractor;

% Recognizer and representer is a simple blob finder.
Recognizer.recognize = @find_blob;
Representer.represent = @filter_blobs;

% The tracker module.
Tracker.H          = eye(4);        % System model
Tracker.Q          = 0.5 * eye(4);  % System noise
Tracker.F          = eye(4);        % Measurement model
Tracker.R          = 5 * eye(4);    % Measurement noise
Tracker.innovation = 0;
Tracker.track      = @kalman_step;

% A custom visualizer for the Kalman state.
Visualizer.visualize = @visualize_kalman;
Visualizer.paused    = false;

% Set up the global tracking system.
T.segmenter   = Segmenter;
T.recognizer  = Recognizer;
T.representer = Representer;
T.tracker     = Tracker;
T.visualizer  = Visualizer;

% And run the tracker on the video.
run_tracker(fpath, T);


function T = run_tracker(fpath, T)
% RUN_TRACKER - toplevel function for starting a tracker on a video.
%
% RUN_TRACKER(fpath, T) will start the tracker represented by the
% structure T running on the video in the file specified by
% 'fpath'. This function handles all video I/O and the threading of
% state through all of the tracking stages.
%
% In the structure T you may set components to be run:
%  T.segmenter   - the SEGMENTER for detecting foreground objects.
%  T.recognizer  - for recognizing targets among foreground objects.
%  T.representer - to represent the recognized target.
%  T.tracker     - to do the actual tracking.
%  T.visualizer  - to visualize the results.
%
% See kalman_tracker for a complete example.
%
% See also: kalman_tracker, find_blob, filter_blobs, kalman_step,
% visualize_kalman.

% load filenames
a = dir([fpath filesep 'exp*']);

T.time         = 0;
T.frame_number = 0;
T.fps          = 250;
T.num_frames   = length(a);

while T.frame_number <= T.num_frames
  T.frame_number = T.frame_number + 1;
  
  % load current frame
  frame = imread([fpath filesep a(T.frame_number).name]);

  if isfield(T, 'segmenter')
    T = T.segmenter.segment(T, frame);
  end
  
  if isfield(T, 'recognizer')
    T = T.recognizer.recognize(T, frame);
  end
  
  if isfield(T, 'representer')
    T = T.representer.represent(T, frame);
  end
  
  if isfield(T, 'tracker')
    T = T.tracker.track(T, frame);
  end
  
  if isfield(T, 'visualizer')
    T = T.visualizer.visualize(T, frame);
  end
  
  T.time = T.time + 1/T.fps;

end
return

function T = background_subtractor(T, frame)
% BACKGROUND_SUBTRACTOR - simple image differencing foreground segmentation.
%
% NOTE: This function is intended to be run as a SEGMENTER in the
% tracking framework.  See documentation for RUN_TRACKER.
%
% BACKGROUND_SUBTRACTOR(T, frame) processes image 'frame',
% segmenting it and storing the segmented image in
% T.segmenter.segmented.  The background model in T.segmenter is
% updated and the current background image stored in
% T.segmenter.background.
%
% Parameters used from T.segmenter:
%  T.segmenter.gamma  - update rate for background model.
%  T.segmenter.tau    - threshold for fore/background segmentation.
%  T.segmenter.radius - radius of closing used on foreground mask.
%
% Inputs:
%  T     - tracker state structure.
%  frame - image to process.

% Do everything in grayscale.
% frame_grey = double(frame);
frame_grey = frame;

% Check to see if we're initialized
if ~isfield(T.segmenter, 'background');
  T.segmenter.background = frame_grey;
end

% Pull local state out.
gamma  = T.segmenter.gamma;
tau    = T.segmenter.tau;
radius = T.segmenter.radius;

% Rolling average update.
T.segmenter.background = gamma * frame_grey + (1 - gamma) * ...
    T.segmenter.background;

% And threshold to get the foreground.
T.segmenter.segmented = abs(T.segmenter.background - frame_grey) > tau;
T.segmenter.segmented = imclose(T.segmenter.segmented, strel('disk', radius));

return

function T = find_blob(T, frame)
% FIND_BLOBS - simple recognizer of targets largest foreground bounding boxes.
%
% NOTE: This function is intended to be run as a RECOGNIZER in the
% tracking framework.  See documentation for RUN_TRACKER.
%
% FIND_BLOBS(T, frame) will recognize foreground objects in the
% current image 'frame'.  The blobs detected in the image are
% stored in T.recognizer.blobs.
%
% Parameters set in T.recognizer:
%  T.recognizer.blobs - the blobs detected in the foreground of frame.
% 
% Inputs:
%  T     - tracker state structure.
%  frame - image to process.
% 
% See also: run_tracker
%
T.recognizer.blobs = bwlabel(T.segmenter.segmented);
return

function T = filter_blobs(T, frame)
% FILTER_BLOBS - simple representer of targets as largest foreground bounding box.
%
% NOTE: This function is intended to be run as a REPRESENTER in the
% tracking framework.  See documentation for RUN_TRACKER.
%
% FILTER_BLOBS(T, frame) will represent the measurement for the
% frame in image 'frame' as the bounding box of the largest
% detected blob in the foreground of the frame.
%
% Parameters used from T.recognizer:
%  T.recognizer.blobs - the blobs detected in the foreground of frame.
% 
% Inputs:
%  T     - tracker state structure.
%  frame - image to process.
% 
% See also: run_tracker

% Make sure at least one blob was recognized
if sum(sum(T.recognizer.blobs))
  % Extract the BoundingBox and Area of all blobs
  R = regionprops(T.recognizer.blobs, 'BoundingBox', 'Area');
  
  % And only keep the biggest one
  [I, IX] = max([R.Area]);
  T.representer.BoundingBox = R(IX(size(IX,2))).BoundingBox;
end
return

function T = kalman_step(T, frame)
% KALMAN_STEP - a single iteration of the Kalman filter.
%
% NOTE: This function is intended to be run as a TRACKER in the
% tracking framework.  See documentation for RUN_TRACKER.
%
% KALMAN_STEP(T, frame) takes the current frame image in 'frame',
% extracts the parameters of the Kalman filter from T.tracker, the
% measurement from T.representer, and advances the Kalman filter
% estimation one step.
%
% Parameters used from T.representer:
%  T.representer.BoundingBox - the bounding box of the current target.
%
% Parameters used from T.tracker:
%  T.tracker.H  - system model
%  T.tracker.Q  - system noise
%  T.tracker.F  - measurement model
%  T.tracker.R  - measurement noise
% 
% Inputs:
%  T     - tracker state structure.
%  frame - image to process.
% 
% See also: run_tracker, kalman_tracker


% Get the current filter state.
K = T.tracker;

% Don't do anything unless we're initialized.
if isfield(K, 'm_k1k1') && isfield(T.representer, 'BoundingBox')

  % Get the current measurement out of the representer.
  z_k = T.representer.BoundingBox';
  
  % Project the state forward m_{k|k-1}.
  m_kk1 = K.F * K.m_k1k1;
  
  % Partial state covariance update.
  P_kk1 = K.Q + K.F * K.P_k1k1 * K.F';
  
  % Innovation is disparity in actual versus predicted measurement.
  innovation = z_k - K.H * m_kk1;
  
  % The new state covariance.
  S_k = K.H * P_kk1 * K.H' + K.R;
  
  % The Kalman gain.
  K_k = P_kk1 * K.H' * inv(S_k);
  
  % The new state prediction.
  m_kk = m_kk1 + K_k * innovation;
  
  % And the new state covariance.
  P_kk = P_kk1 - K_k * K.H * P_kk1;
  
  % Innovation covariance.
  K.innovation = 0.2 * sqrt(innovation' * innovation) + (0.8) ...
      * K.innovation;
  
  % And store the current filter state for next iteration.
  K.m_k1k1 = m_kk;
  K.P_k1k1 = P_kk;
else
  if isfield(T.representer, 'BoundingBox');
    K.m_k1k1 = T.representer.BoundingBox';
    K.P_k1k1 = eye(4);
  end
end

% Make sure we stuff the filter state back in.
T.tracker = K;

return

function T = visualize_kalman(T, frame)
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

% Display the current frame.
image(frame);

% Draw the current measurement in red.
if isfield(T.representer, 'BoundingBox')
  rectangle('Position', T.representer.BoundingBox, 'EdgeColor', 'r');
end

% And the current prediction in green
if isfield(T.tracker, 'm_k1k1');
  rectangle('Position', T.tracker.m_k1k1, 'EdgeColor', 'g');
end
drawnow;

% If we're paused, wait (but draw).
while (getappdata(gcf, 'paused'))
  drawnow;
end
return

% This is a callback function that toggles the pause state.
function pauseHandler(a, b, h)
setappdata(h, 'paused', xor(getappdata(h, 'paused'), true));
return

