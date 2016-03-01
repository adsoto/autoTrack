function T = run_tracker(fname, T)
% RUN_TRACKER - toplevel function for starting a tracker on a video.
%
% RUN_TRACKER(fname, T) will start the tracker represented by the
% structure T running on the video in the file specified by
% 'fname'. This function handles all video I/O and the threading of
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
vr = videoReader(fname);

% load filenames
% a = dir([fname filesep 'exp*']);

T.time         = 0;
T.frame_number = 0;
T.fps          = getfield(get(vr), 'FrameRate');
T.num_frames   = getfield(get(vr), 'NumberOfFrames');

% T.fps          = 250;
% T.num_frames   = length(a);

% while nextFrame(vr)
while T.frame_number <= T.num_frames
  T.frame_number = T.frame_number + 1;
  
  % load current frame
%   frame = imread([fname filesep a(T.frame_number).name]);
  
  frame = getFrame(vr);

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
