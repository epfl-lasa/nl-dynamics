
function [x_hat P_hat]=KalmanMeasurementUpdate(x_pred,P_pred,y,C,R)
%[x_hat P_hat]=KalmanMeasurementUpdate(x_pred,P_pred,y,C,R)
% Measurement update for the Kalman filter. 
% Inputs:
% x_pred Dx1 predicted state
% P_pred, DxD matrix, covaraince of predicted state
% y Lx1, L-dimensional measurement vector
% C LxD matrix, sensor model
% R LxL measurement noise covariance matrix
% Outputs:
% x_hat, Dx1, the estimated state vector
% P_hat, DxD, covariance of the estimated state vector

H = inv(C*P_pred*C'+R);
L = P_pred*C'*H;
x_hat = x_pred+L*(y-C*x_pred);
P_hat = P_pred - L*C*P_pred;
