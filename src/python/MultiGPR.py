#!/usr/bin/env python
from numpy import *
import time

class MultiGPR:
    def __init__(self, inputDim, outputDim):
        self.inputData = []
        self.outputData = []

        self.KXX = None
        self.KXX_ = None
        self.KXx = None
        self.KxX = None

        self.inputDim = inputDim
        self.outputDim = outputDim
        self.nData = 0

        self.l_scale = 0.0
        sigma_f = 0.0
        sigma_n = 0.0

        dist = []
        regressors = []

    # Sets the hyperparameters for the GPR
    def setHyperParams(self, l, f, n):
        self.l_scale = l
        self.sigma_f = f
        self.sigma_n = n

    # Add new training data
    def addTrainingData(self, new_in, new_out):
        self.nData += 1
        if (size(new_in) == self.inputDim and size(new_out) == self.outputDim):
            self.inputData.append(new_in)
            self.outputData.append(new_out)
        else:
            print 'Error, dimension mismatch.'

    def removeTrainingData(self, start, end):#old_in, old_out):
        print 'Starting size = ' + str(self.nData)
        self.nData = self.nData - (end-start+1)#self.nData - 1
        print 'Ending size = ' + str(self.nData)

        for i in range(end,start-1,-1):
            #print 'Removing index ' + str(i)
            self.inputData.pop(i)
            self.outputData.pop(i)

        return True

        #indices_in = [i for i, x in enumerate(self.inputData) if (x == old_in).all()]
        #indices_out = [i for i, x in enumerate(self.outputData) if (x == old_out).all()]

        #indices = indices_in#indices = list(set(indices_in).intersection(indices_out))
        #if size(indices) == 0:
        #    print 'Error, not found.'
        #    return
        #elif size(indices) > 1:
        #    indices = indices[size(indices)-1]
        #else:
        #    indices = indices[0]

        #self.inputData.pop(indices)
        #self.outputData.pop(indices)

    # Squared Exponential Covariance Function
    def SQEcovFuncD(self, x1, x2):
        dist = x1-x2
        d = dist.dot(dist)
        d = pow(self.sigma_f,2)*exp(-d/(pow(self.l_scale,2)*2))
        return d

    # Vectorized Squared Exponential Covariance Function
    def SQEcovFuncDV(self, x1, x2):
        dist = x1-x2
        d = einsum('ij,ij->i', dist,dist)
        d = pow(self.sigma_f,2)*exp(-d/(pow(self.l_scale,2)*2))
        return d

    # Call SQEcovFuncD on all the input data
    def SQEcovFunc(self, x1=None, x2=None):
        #start = time.time()
        if x1 !=None and x2 !=None:
            n = len(x1)
            KXx = zeros(n)
            for i in range(0, n):
                KXx[i] = self.SQEcovFuncD(x1[i],x2)
            return KXx
        elif x1 !=None:
            n = len(x1)
            retMat = zeros((n,n))
            for i in range(0, n):
                for j in range(i, n):
                    retMat[i,j] = self.SQEcovFuncD(x1[i],x1[j])
                    retMat[j,i] = retMat[i,j]
            return retMat
        else:
            print 'Error, incorrect number of arguments.'
            return None

    def sq_dist(self,a,b):
        am = array([mean(a,1)]).transpose()
        bm = array([mean(b,1)]).transpose()
        a = a-am
        b = b-bm
        C = -2*dot(transpose(a),b)
        C += transpose(array([sum(a*a,0)])) + array([sum(b*b,0)])
        return C

    def sq_dist_centered(self,a,b):
        C = -2*dot(transpose(a),b)
        C += transpose(array([sum(a*a,0)])) + array([sum(b*b,0)])
        return C

#    def covSEiso(self,a,b):
#      # compute squared distance:
#        C = self.sq_dist(transpose(a)/self.l_scale,transpose(b)/self.l_scale)
#        K = self.sigma_f*exp(-C/2)
#        return K

    def covSEiso(self,a,b=None):
        a = transpose(a)
        if(b==None):
            return self.covSEiso_symmetric(a)
        # sort of centering of data to increase numerical robustness
        m_a,n_a = shape(a)
        m_b,n_b = shape(b)
        dm = n_a/(n_a+n_b)*array([mean(a,1)]).transpose()
        dm += n_b/(n_a+n_b)*array([mean(b,1)]).transpose()
        a = a-dm
        b = b-dm
        # compute squared distance:
        C = self.sq_dist_centered(a/self.l_scale,b/self.l_scale)
        K = self.sigma_f*exp(-C/2)
        return K

    def covSEiso_symmetric(self,a):
        # centering data for increased robustness:
        am = array([mean(a,1)]).transpose()
        a = a-am
        # compute squared sistance:
        C = self.sq_dist_centered(a/self.l_scale,a/self.l_scale)
        K = self.sigma_f*exp(-C/2)
        return K

    # Time consuming
    def prepareRegression(self):
        if len(self.inputData) <=1:
            print 'Error. Not enough data.'
        # Time consuming
        self.KXX = self.covSEiso(self.inputData)
        #self.covSEiso(self.inputData, self.inputData)
        #self.SQEcovFunc(x1= self.inputData)
        self.KXX_ = copy(self.KXX)
        for i in range(0, size(self.KXX[0])):
            self.KXX_[i,i] += self.sigma_n*self.sigma_n
        self.KXX_ = linalg.inv(self.KXX_)

    def doRegression(self, inp):
        outputData = transpose(matrix(self.outputData))
        outp = zeros(size(outputData[:,0]))
        self.KXx = transpose(self.covSEiso(asarray(self.inputData), transpose(array([inp]))))
        tmp = dot(self.KXX_,self.KXx[0])#transpose(matrix(array(self.KXX_)) * transpose(matrix(array(self.KXx))))
        for i in range(0, outputData[:,0].size):
            outp[i] = dot(tmp, transpose(outputData[i]))
        return outp
