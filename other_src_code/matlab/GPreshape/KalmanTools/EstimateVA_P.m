

function [X_hat,Xd_hat,Xdd_hat]=EstimateVA_P(X,dt,Q,R,xi0,P0)

% [X_hat,Xd_hat,Xdd_hat]=EstimateVA_P(X,dt,Q,R,xi0,P0)
% this function uses Forward-Backward Kalman smoothing for producing 
% estimates of position, velocity and acceleration profiles using
% position measurements only. The result is consistent, in the sense that
% if the estimated acceleration is integrated, it results in the estimated 
%velocity and position. A constant acceleration model is assumed,
% with changes to the acceleration arrising from Gaussian process noise
% with variance for each dimension given by Q.
%
%Inputs:
% X, a DXN matrix containing N samples of D-dimensional position
% measurements
% dt, a scalar representing the sample time
% Q, a Dx1 Vector containing the process noise for each dimension. A high
% value 'allows' the acceleration to change more rapidly. 
% R, a DX1 Vector containing the measurement noise variance for each
% dimension
% xi0, a 3DX1 Vector containing the guesstimated position, velocity and 
% acceleration at time=0.Ex: xi0=[x0;y0;xd0;yd0;xdd0;ydd0]
% P0, a 3DX3D matrix containing the covariance of the initial guesstimate. 
%Outputs:
% X_hat, a DxN matrix containing the position estimates.
% Xd_hat, a DxN matrix containing the velocity estimates.
% Xdd_hat, a DxN matrix containing the acceleation estimates.


% check the inputs:
[dim,Ndata] = size(X);
if (dim>Ndata)
    error('The number of dimensions are > the number of data. You should give measurements as a DxN matrix!')
end
if(length(dt)~=1)
    error('dt should be a scalar!')
end
if(length(Q)~=dim)
    error('Q should be a vector with the process noise of the acceleration in each dimension. Q should be Dx1!')
end
if(length(R)~=dim)
    error('R should be a vector with the measurement noise variance on position in each dimension. R should be Dx1!')
end
if(length(xi0)~=3*dim)
    error('xi0 should be a vector with the initial state guesstimate. xi0 should be 3Dx1!')
elseif(size(xi0,2)>size(xi0,1))
    xi0=xi0';
end
if(size(P0,1)~=3*dim | size(P0,2)~=3*dim)
    error('P0 should be a 3Dx3D covariance matrix of xi0!')
end
Qvec=Q;
Q = zeros(3*dim,3*dim);
R=diag(R);
%create system matrix A, sensor model C, and process noise covariance Q
A = zeros(3*dim,3*dim);
C = zeros(dim,3*dim);
for i=1:dim
    % A
    posRow = zeros(1,3*dim);
    posRow(i)=1;
    posRow(dim+i)=dt;
    posRow(2*dim+i)=dt^2/2;
    A(i,:)=posRow;
    velRow = zeros(1,3*dim);
    velRow(i+dim)=1;
    velRow(i+2*dim)=dt;
    A(dim+i,:)=velRow;
    accRow = zeros(1,3*dim);
    accRow(2*dim+i)=1;
    A(2*dim+i,:)=accRow;
    % C
    crow = zeros(1,3*dim);
    crow(i)=1;
    C(i,:)=crow;
    % Q
    Q(2*dim+i,2*dim+i)=Qvec(i);
end

% forward filtering
[xi_pred,P_pred,xi_hat,P_hat]=KalmanPredict(xi0,P0,X(:,1),A,C,Q,R);
Xi_pred = xi_pred;
Xi_hat = xi_hat;
P_pred_=zeros(3*dim,3*dim,Ndata);
P_pred_(:,:,1)=P_pred;
P_hat_=zeros(3*dim,3*dim,Ndata);
P_hat_(:,:,1)=P_hat;
for i=2:Ndata
   [xi_pred,P_pred,xi_hat,P_hat]=KalmanPredict(xi_pred,P_pred,X(:,i),A,C,Q,R);
   Xi_pred = [Xi_pred,xi_pred];
   Xi_hat = [Xi_hat,xi_hat];
   P_pred_(:,:,i)=P_pred;
   P_hat_(:,:,i)=P_hat;   
end

% backward filtering
Xi_hat_2=Xi_hat;
P_b_=P_hat_;
for i=1:Ndata-1
   j=Ndata-i;
   Xi_hat_2(:,j)= KalmanBackwardStep(Xi_hat(:,j),P_hat_(:,:,j),Xi_pred(:,j),P_pred_(:,:,j),Xi_hat_2(:,j+1),P_b_(:,:,j+1),A);
end

X_hat = Xi_hat_2(1:dim,:);
Xd_hat = Xi_hat_2(dim+1:2*dim,:);
Xdd_hat = Xi_hat_2(2*dim+1:end,:);










