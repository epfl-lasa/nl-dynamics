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
    size=len(pos)
    #size = max(len(pos), len(vel))
    data = Spline3D()
    t = np.linspace(0,1,size)

    #data.coord=[spline_(t, pos[:,i], vel[:,i]) for i in range(3)]
    x=spline_(t, pos[:,0], vel[:,0])
    print(x.coef[size-1])

    y=spline_(t, pos[:,1], vel[:,1])
    print(y.coef[size-1])

    z=spline_(t, pos[:,2], vel[:,2])
    print(z.coef[size-1])


    #data.coord=[spline_(t, pos[:,0], vel[:,0]), spline_(t, pos[:,1], vel[:,1]), spline_(t, pos[:,2], vel[:,2])]
    data.coord=[x,y,z]

    #print(data.coord[0].coef[-1])
    #print(data.coord[1].coef[-1])
    #print(data.coord[2].coef[-1])

    # print(x.coef[-1])
    # print(y.coef[-1])
    # print(z.coef[-1])

    return data

def getPointsSpline3D(splineData3D, t):
    return [ getPointsSpline(splineData3D.coord[i], t) for i in range(3) ]

def spline_(x,y,derivate):
    #print("%lf %lf" % (min(y), max(y)))
    data = Spline()
    data.nbPoints = len(x)-1

    for i in range(data.nbPoints) :
        data.coef.append(systemSolver(x[i], x[i+1], y[i], y[i+1], derivate[i], derivate[i+1]))
        data.points.append( [ x[i], x[i+1] ] )

    print(data.coef[-1])
    #print("t=0 y=%f, t=1 y=%f, %f*t^3+%f*t^2+%f*t+%f ; %f*t^3+%f*t^2+%f*t+%f" % (y[0], y[-1], data.coef[0][0],data.coef[0][1],data.coef[0][2],data.coef[0][3],data.coef[-1][0],data.coef[-1][1],data.coef[-1][2],data.coef[-1][3]))
    return data

def systemSolver(t1, t2, f1, f2, fp1, fp2):



    A = [[pow(t1,3),   pow(t1,2),   t1, 1],
         [pow(t2,3),   pow(t2,2),   t2, 1],
         [3*pow(t1,2), 2*t1,        1,  0],
         [3*pow(t2,2), 2*t2,        1,  0]]

    b = [[f1],[f2],[fp1],[fp2]]

    return np.linalg.solve(A,b)

def getPointsSpline(splineData, xx):
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

