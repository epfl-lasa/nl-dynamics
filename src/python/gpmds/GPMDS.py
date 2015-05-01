#!/usr/bin/env python
from numpy import *
import numpy as np
import math

from gpmds.MultiGPR import MultiGPR


class GPMDS:
    def __init__(self, ell, sigmaF, sigmaN, velocity_cap=None):
        self.ell = ell
        self.sigmaF = sigmaF
        self.sigmaN = sigmaN

        self.mGPR = MultiGPR(2,2)

        self.set_gp_parameters(ell, sigmaF, sigmaN)
        if not velocity_cap:
            velocity_cap = (0.1, 1.0)

        self.v_capLow = velocity_cap[0]
        self.v_capHigh = velocity_cap[1]



    def set_gp_parameters(self, ell, sigmaF, sigmaN):
        self.mGPR.setHyperParams(ell, sigmaF, sigmaN)

    def setOriginalDynamics(self, fun):
        # Set the original dynamics to the user passed function
        self.originalDynamics = fun

    def addData(self, position, velocity):
        theta = self.getTheta(position, velocity)
        # Add the data point to training data
        self.mGPR.addTrainingData(position, theta)

    def addDataM(self, position, velocity):
        for i in range(0,size(position[0])):
            self.addData(array([position[0,i],position[1,i]]), array([velocity[0,i],velocity[1,i]]))

        self.mGPR.prepareRegression()


    def getTheta(self, position, velocity):
        # Compute original dynamics at position
        originalVelocity = self.originalDynamics(position)
        # Find scaling between vectors
        kappa = (linalg.norm(velocity)/linalg.norm(originalVelocity))-1.0

        # Compare original velocity with velocity to compute speed scaling (kappa) and rotation axis angle
        # Check for divide by 0 error
        if linalg.norm(velocity)==0:
            normDesDir = velocity
        else:
            normDesDir = velocity/linalg.norm(velocity)

        #print originalVelocity
        if linalg.norm(originalVelocity)==0:
            normOrgDir = originalVelocity
        else:
            normOrgDir = originalVelocity/linalg.norm(originalVelocity)

        # Calculate angle of rotation
        angle = math.atan2(normDesDir[1],normDesDir[0])-math.atan2(normOrgDir[1],normOrgDir[0])

        # put in a good range
        if(angle > pi):
            angle = -(2*pi-angle)
        elif(angle < -pi):
            angle = 2*pi+angle;

        # Create the datapoint, theta (the 2 number output of the GPR- angle, scaling)
        theta = zeros(2)
        theta[0] = angle
        theta[1] = kappa

        return theta


    def removeData(self, start, end):#position, velocity):
        #print 'Removing data...'
        #theta = self.getTheta(position, velocity)
        s =  self.mGPR.removeTrainingData(start, end)#self.mGPR.removeTrainingData(position,theta)
        if len(self.mGPR.inputData) > 0:
            self.mGPR.prepareRegression()
        return s

    def reshapedDynamics(self, position):
        # Get original dynamics at position
        originalVelocity = self.originalDynamics(position)
        if self.mGPR.nData==0:
            velocity = originalVelocity
            #if(linalg.norm(velocity)>self.v_capHigh):
            #    velocity = velocity/linalg.norm(velocity)*self.v_capHigh
            #if (linalg.norm(velocity)<self.v_capLow):
            #    velocity = velocity/linalg.norm(velocity)*self.v_capLow
            #return velocity
        else:
            result = self.mGPR.doRegression(position)
            angle = result[0]
            kappa = result[1]
            kappa = max(kappa, -0.9)

            # Calculate the rotation matrix
            R = array([[cos(angle), -sin(angle)],[sin(angle),cos(angle)]])
            # Rotate velocity by R and scale by kappa
            velocity = (1.0 + kappa)*dot(R, originalVelocity)

        # Put velocity limits
        if (linalg.norm(velocity)>self.v_capHigh):
            velocity = velocity/linalg.norm(velocity)*self.v_capHigh
        if (linalg.norm(velocity)<self.v_capLow):
            velocity = velocity/linalg.norm(velocity)*self.v_capLow
        return velocity



def originalDynamicsLinear(pose):
    p = 4
    A = -p * np.eye(2)
    xd = np.dot(A, pose)
    jacobian = A
    return xd
# function [xd jacobian] = originalDynamicsLINEAR(x)
# % stable linear isotropic dynamics. adjust speed of convergence with p.
# p = 4;
# A = -p*eye(2);
# xd =A*x;
# %xd(2,:) =xd(2,:) + 2*p*x(1,:).*cos(2*pi/150*x(1,:));
# jacobian = A;


if __name__ == '__main__':
    print 'hello'

    (ell, sigmaF, sigmaN) = (30, 1.0, 0.4)
    gp = GPMDS(ell, sigmaF, sigmaN)
    print gp
    print gp.mGPR

    originalDynamics = originalDynamicsLinear

    pose = np.array((5, 2))
    velocity = np.array((1.0, 1.0))

    gp.setOriginalDynamics(originalDynamics)

    gp.addData(pose, velocity)
    gp.mGPR.prepareRegression()

    th = gp.getTheta(pose, velocity)
    print 'Th: {}'.format(th)

    reshaped = gp.reshapedDynamics(pose)
    print 'Reshaped: {}'.format(reshaped)


