function [x_pred P_pred x_hat P_hat]=KalmanPredict(x_pred,P_pred,y,A,C,Q,R)
%[x_pred P_pred x_hat P_hat]=KalmanPredict(x_pred,P_pred,y,A,C,Q,R)
% performs one step measurement update and one time update of the kalman
% filter.
% Inputs:
% x_pred, Dx1 predicted state vector (from previous step, or x0)
% P_pred, DxD state prediction covariance (from previous step, or P0)
% y, LX1, L-dimensional measurement vector
% A, DxD system matrix
% C, LxD sensor model
% Q, DxD process noise covariance matrix
% R, LxL measurement noise covariance matrix
%
%Outputs:
% x_pred, Dx1 state prediction
% P_pred, DxD prediction covaraince matrix
% x_hat, Dx1 state estimate
% P_hat, DxD covaraince matrix of state estimate

[x_hat,P_hat]=KalmanMeasurementUpdate(x_pred,P_pred,y,C,R);
[x_pred,P_pred]=KalmanTimeUpdate(x_hat,P_hat,A,Q);











