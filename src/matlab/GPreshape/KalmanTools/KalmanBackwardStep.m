function [x_b P_b]=KalmanBackwardStep(x_hat,P_hat,x_pred,P_pred,x_b_front,P_b_front,A)
% Backward step for the Rauch-Tung-Striebel Algorithm for fixed interval
% smoothing

i_P_pred = inv(P_pred);
x_b = x_hat+P_hat*A'*i_P_pred*(x_b_front-x_pred);
P_b = P_hat+P_hat*A'*i_P_pred*(P_b_front-P_pred)*i_P_pred*A*P_hat;



   