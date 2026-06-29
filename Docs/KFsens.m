classdef KFsens < matlab.System
    properties
        % R - measurement noise covariance matrix
        % H - system measurement matrix
        % z - measurement vector
        H = [0 0 0];
        R = 1e3;
    end
end
