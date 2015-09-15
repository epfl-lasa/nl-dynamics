function [x_pred P_pred]=KalmanTimeUpdate(x_hat,P_hat,A,Q)
%[x_pred P_pred]=KalmanTimeUpdate(x_hat,P_hat,A,Q)
% Time update of the Kalman filter. 
% Inputs:
% x_hat, Dx1 current estimated state vector
% P_hat, DxD covaraiance matrix of current state estimate
% A DxD system matrix
% Q DxD covaraince of process noise
% Outputs:
% x_pred, Dx1 predicted next state
% P_pred, DxD covariance matrix of state prediction
x_pred = A*x_hat;
P_pred = A*P_hat*A'+Q;