import numpy as np


# spline and spline function assume that we are using a simple system as y=f(x)
class Spline(object):
    def __init__(self):
        self.points = []
        self.coef = []
        self.nbPoints = 0


# spline3D function assume that we are using a parametrise system
# as (x,y,z) = f(t)
class Spline3D:
    def __init__(self):
        self.coord = [Spline(), Spline(), Spline()]

    def get_spline(self, element):
    # getter function, state = 0,1,2 or 'x','y','z' for x,y,z splines
        if isinstance(element, int) and element >= 0 and element <= 2:
            return self.coord[element]
        elif isinstance(element, str):
            if element == 'x':
                return self.coord[0]
            elif element == 'y':
                return self.coord[1]
            elif element == 'z':
                return self.coord[2]


def spline3D(data):
    #data is a KUKA/CartStamped structur:
    # data = array /
                    # header
                    #   seq
                    #   stamp
                    #     secs
                    #     nsecs
                    #   frame_id
                    # pose
                    #   position
                    #     x
                    #     y
                    #     z
                    #   orientation
                    #     x
                    #     y
                    #     z
                    #     w
                    # twist
                    #   linear
                    #     x
                    #     y
                    #     z
                    #   angular
                    #     x
                    #     y
                    #     z
                    # wrench
                    #   force
                    #     x
                    #     y
                    #     z
                    #   torque
                    #     x
                    #     y
                    #     z

    # creating data output
    spline_data = Spline3D()
    t = np.linspace(0, 1, len(data))

    #splines axis projection calculation
    spline_data.coord[0] = spline_(t, [d.pose.position.x for d in data],
                                      [d.twist.linear.x for d in data])

    spline_data.coord[1] = spline_(t, [d.pose.position.y for d in data],
                                      [d.twist.linear.y for d in data])

    spline_data.coord[2] = spline_(t, [d.pose.position.z for d in data],
                                      [d.twist.linear.z for d in data])

    return spline_data


def spline3Dvect(pos, vel):
    # creating data output
    data = Spline3D()
    t = np.linspace(0, 1, len(pos))

    #splines axis projection calculation
    data.coord[0] = spline_(t, pos[:, 0], vel[:, 0])
    data.coord[1] = spline_(t, pos[:, 1], vel[:, 1])
    data.coord[2] = spline_(t, pos[:, 2], vel[:, 2])

    return data


def getPointsSpline3D(splineData3D, t):
    return [getPointsSpline(splineData3D.coord[i], t) for i in range(3)]


def spline_(x, y, derivate):
    #creating data output
    data = Spline()
    data.nbPoints = len(x)-1

     #spline data calculation
    for i in range(data.nbPoints):
        data.coef.append(systemSolver(x[i], x[i+1], y[i], y[i+1],
                         derivate[i], derivate[i+1]))
        data.points.append([x[i], x[i+1]])

    return data


def systemSolver(t1, t2, f1, f2, fp1, fp2):
    A = [[t1**3,     t1**2,  t1, 1],
         [t2**3,     t2**2,  t2, 1],
         [3*(t1**2), 2*t1,   1,  0],
         [3*(t2**2), 2*t2,   1,  0]]

    b = [[f1], [f2], [fp1], [fp2]]

    return np.linalg.solve(A, b)


def getPointsSpline(splineData, xx):
    #creating data output
    yy = []

    count = 0
    save = xx[0]-1
    for x in xx:

        if x < save:
            count = 0

        while count < splineData.nbPoints and x > splineData.points[count][1]:
            count += 1

        save = x
        yy.extend(getValCoef(splineData.coef[count], x))

    return yy


def getValCoef(coef, x):
    return coef[0]*(x**3)+coef[1]*(x**2)+coef[2]*x+coef[3]

__author__ = 'sballmer'
