import numpy as np

# spline and spline function assume that we are using a simple system as y=f(x)
class Spline:
    points=[]
    coef=[]
    nbPoints=0

# spline3D and spline3D function assume that we are using a parametrise system as (x,y,z) = f(t)
class Spline3D:
    coord=[Spline(), Spline(), Spline()]


def spline3D(pos,vel):
    # creating data output
    size=len(pos)
    data = Spline3D()
    t = np.linspace(0,1,size)

    #splines axis projection calculation
    x=spline_(t, pos[:,0], vel[:,0])
    y=spline_(t, pos[:,1], vel[:,1])
    z=spline_(t, pos[:,2], vel[:,2])

    data.coord=[x,y,z]

    return data

def getPointsSpline3D(splineData3D, t):
    return [ getPointsSpline(splineData3D.coord[i], t) for i in range(3) ]

def spline_(x,y,derivate):
    #creating data output
    data = Spline()
    data.coef=[]
    data.points=[]
    data.nbPoints = len(x)-1

    #spline data calculation
    for i in range(data.nbPoints) :
        data.coef.append(systemSolver(x[i], x[i+1], y[i], y[i+1], derivate[i], derivate[i+1]))
        data.points.append( [ x[i], x[i+1] ] )

    return data

def systemSolver(t1, t2, f1, f2, fp1, fp2):
    A = [[t1**3,     t1**2,  t1, 1],
         [t2**3,     t2**2,  t2, 1],
         [3*(t1**2), 2*t1,   1,  0],
         [3*(t2**2), 2*t2,   1,  0]]

    b = [[f1],[f2],[fp1],[fp2]]

    return np.linalg.solve(A,b)

def getPointsSpline(splineData, xx):
    #creating data output
    yy=[];

    count = 0
    save=xx[0]-1
    for x in xx :

        if x<save :
            count = 0;

        while count<splineData.nbPoints and x > splineData.points[count][1] :
            count += 1

        save = x
        yy.extend(getValCoef(splineData.coef[count], x))

    return yy

def getValCoef(coef, x):
    return coef[0]*(x**3)+coef[1]*(x**2)+coef[2]*x+coef[3]



__author__ = 'sballmer'

