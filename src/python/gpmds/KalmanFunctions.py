from numpy import *

def KalmanPredict(x_pred, P_pred, y, A, C, Q, R):
    # [x_pred P_pred x_hat P_hat]=KalmanPredict(x_pred,P_pred,y,A,C,Q,R)
    # performs one step measurement update and one time update of the kalman filter.
    # Inputs:
    # x_pred- Dx1 predicted state vector (from previous step, or x0)
    # P_pred- DxD state prediction covariance (from previous step, or P0)
    # y- LX1 L-dimensional measurement vector
    # A- DxD system matrix
    # C- LxD sensor model
    # Q- DxD process noise covariance matrix
    # R- LxL measurement noise covariance matrix

    # Outputs:
    # x_pred- Dx1 state prediction
    # P_pred- DxD prediction covaraince matrix
    # x_hat- Dx1 state estimate
    # P_hat- DxD covaraince matrix of state estimate
    x_hat, P_hat = KalmanMeasurementUpdate(x_pred,P_pred,y,C,R)
    x_pred,P_pred = KalmanTimeUpdate(x_hat,P_hat,A,Q)
    return x_pred, P_pred, x_hat, P_hat


def KalmanMeasurementUpdate(x_pred,P_pred,y,C,R):
    # [x_hat P_hat]=KalmanMeasurementUpdate(x_pred,P_pred,y,C,R)
    # Measurement update for the Kalman filter.

    # Inputs:
    # x_pred- Dx1 predicted state
    # P_pred- DxD matrix, covaraince of predicted state
    # y- Lx1, L-dimensional measurement vector
    # C- LxD matrix, sensor model
    # R- LxL measurement noise covariance matrix

    # Outputs:
    # x_hat- Dx1, the estimated state vector
    # P_hat- DxD, covariance of the estimated state vector
    H = linalg.inv(dot(dot(C,P_pred),transpose(C))+R)#linalg.inv(array(matrix(C)*matrix(P_pred)*matrix(transpose(C)))+R)#linalg.inv(dot(dot(C,P_pred),transpose(C))+R)
    L = dot(dot(P_pred,transpose(C)),H)#array(matrix(P_pred)*matrix(transpose(C))*matrix(H))#dot(dot(P_pred,transpose(C)),H)
    if len(x_pred) == 1:
        x_pred = transpose(x_pred)
    x_hat = x_pred+dot(L,(transpose([y])-dot(C,x_pred)))
    #array(matrix(transpose(x_pred))+matrix(L)*(matrix(transpose(y))-matrix(C)*matrix(transpose(x_pred))))#x_pred+dot(L,(y-dot(C,x_pred)))
    P_hat = array(matrix(P_pred)-matrix(L)*matrix(C)*matrix(P_pred))#P_pred - dot(dot(L,C),P_pred)
    return transpose(x_hat), P_hat


def KalmanTimeUpdate(x_hat, P_hat, A, Q):
    # [x_pred P_pred]=KalmanTimeUpdate(x_hat,P_hat,A,Q)
    # Time update of the Kalman filter.
    # Inputs:
    # x_hat- Dx1 current estimated state vector
    # P_hat- DxD covaraiance matrix of current state estimate
    # A- DxD system matrix
    # Q- DxD covaraince of process noise

    # Outputs:
    # x_pred- Dx1 predicted next state
    # P_pred- DxD covariance matrix of state prediction
    x_pred = dot(A,transpose(x_hat))
    P_pred = dot(dot(A,P_hat),transpose(A))+Q
    return x_pred, P_pred


def KalmanBackwardStep(x_hat,P_hat,x_pred,P_pred,x_b_front,P_b_front,A):
    # function [x_b P_b]=KalmanBackwardStep(x_hat,P_hat,x_pred,P_pred,x_b_front,P_b_front,A)
    # Backward step for the Rauch-Tung-Striebel Algorithm for fixed intervalsmoothing

    i_P_pred = linalg.inv(P_pred)
    x_b = x_hat+dot(dot(dot(P_hat,transpose(A)),i_P_pred),(x_b_front-x_pred))
    P_b = P_hat+dot(dot(dot(P_hat,transpose(A)),dot(i_P_pred,(P_b_front-P_pred))),dot(dot(i_P_pred,A),P_hat))
    return x_b, P_b


def EstimateVA_P(X, dt, Q, R, xi0, P0):
    # this function uses Forward-Backward Kalman smoothing for producing estimates of position, velocity and acceleration profiles using
    # position measurements only. The result is consistent, in the sense that if the estimated acceleration is integrated, it results in the estimated
    # velocity and position. A constant acceleration model is assumed, with changes to the acceleration arrising from Gaussian process noise
    # with variance for each dimension given by Q.

    # Inputs:
    # X, a DXN matrix containing N samples of D-dimensional position measurements
    # dt, a scalar representing the sample time
    # Q, a Dx1 Vector containing the process noise for each dimension. A high value 'allows' the acceleration to change more rapidly.
    # R, a DX1 Vector containing the measurement noise variance for each dimension
    # xi0, a 3DX1 Vector containing the guesstimated position, velocity and acceleration at time=0.Ex: xi0=[x0;y0;xd0;yd0;xdd0;ydd0]
    # P0, a 3DX3D matrix containing the covariance of the initial guesstimate.

    # Outputs:
    # X_hat, a DxN matrix containing the position estimates.
    # Xd_hat, a DxN matrix containing the velocity estimates.
    # Xdd_hat, a DxN matrix containing the acceleration estimates.

    # check the inputs: (dim =row, Ndata=col)

    #print X
    #print dt
    #print Q
    #print R
    #print xi0
    #print P0


    dim = size(X[:,0])
    Ndata = size(X[0])
    if (dim > Ndata):
        print 'The number of dimensions are > the number of data. You should give measurements as a DxN matrix!'
        return None, None, None
    if (size(dt)!=1):
        print 'dt should be a scalar!'
        return
    if (size(Q)!=dim):
        print 'Q should be a vector with the process noise of the acceleration in each dimension. Q should be Dx1!'
        return
    if (size(R)!=dim):
        print 'R should be a vector with the measurement noise variance on position in each dimension. R should be Dx1!'
        return

    if (size(xi0)!=3*dim):
        print 'xi0 should be a vector with the initial state guesstimate. xi0 should be 3Dx1!'
        return
    elif (size(xi0[:,0])>size(xi0[0])):
        xi0=transpose(xi0)

    if (size(P0[:,0])!=3*dim or size(P0[0])!=3*dim):
        print 'P0 should be a 3Dx3D covariance matrix of xi0!'
        return

    Qvec = copy(Q)
    Q = zeros((3*dim,3*dim))
    R = diagflat(R)
    #create system matrix A, sensor model C, and process noise covariance Q
    A = zeros((3*dim,3*dim))
    C = zeros((dim,3*dim))

    for i in range(0,dim):
        # A
        posRow = zeros(3*dim)
        posRow[i] = 1
        posRow[dim+i] = dt
        posRow[2*dim+i] = pow(dt,2)/2.
        A[i,:] = posRow

        velRow = zeros(3*dim)
        velRow[i+dim] = 1
        velRow[i+2*dim] = dt
        A[dim+i,:] = velRow

        A[2*dim+i,2*dim+i] = 1

        # C
        C[i,i] = 1

        # Q
        Q[2*dim+i,2*dim+i] = Qvec[i]
##

    # forward filtering
    xi_pred, P_pred, xi_hat, P_hat = KalmanPredict(xi0, P0, X[:,0], A, C, Q, R)
    Xi_pred = copy(xi_pred)
    Xi_hat = copy(xi_hat)
    P_pred_ = zeros((3*dim,3*dim,Ndata))

    P_pred_[:,:,0] = copy(P_pred)
    P_hat_ = zeros((3*dim,3*dim,Ndata))
    P_hat_[:,:,0] = copy(P_hat)


    for i in range(1,Ndata):
        xi_pred, P_pred, xi_hat, P_hat = KalmanPredict(xi_pred, P_pred, X[:,i], A, C, Q, R)
        Xi_pred = concatenate((Xi_pred,xi_pred), axis=1)
        Xi_hat = concatenate((Xi_hat,xi_hat), axis=0)
        P_pred_[:,:,i] = copy(P_pred)
        P_hat_[:,:,i] = copy(P_hat)

    # backward filtering
    Xi_hat_2 = copy(Xi_hat)
    P_b_ = copy(P_hat_)

    Xi_hat_2 = transpose(Xi_hat_2)
    Xi_hat = transpose(Xi_hat)
    for i in range(0,Ndata-1):
        j = Ndata-i-2
        x_b, P_b = KalmanBackwardStep(Xi_hat[:,j], P_hat_[:,:,j], Xi_pred[:,j], P_pred_[:,:,j], Xi_hat_2[:,j+1], P_b_[:,:,j+1], A)
        Xi_hat_2[:,j] = x_b

    X_hat = Xi_hat_2[0:dim,:]
    Xd_hat = Xi_hat_2[dim:2*dim,:]
    Xdd_hat = Xi_hat_2[2*dim:size(Xi_hat_2[:,0]),:]

    return X_hat, Xd_hat, Xdd_hat
