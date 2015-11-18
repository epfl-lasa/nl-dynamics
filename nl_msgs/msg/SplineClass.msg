# this is a spline class, so it represent a spline interpolation like y=f(x) with points (x1,y1), (x2,y2)
# it means that if this class is used for 3Dspline, the x will be a t-variable parametrisation points will be (t1,x1),(t2,x2) with x an axis variable (can be x,y or z)
# the z field of Point is set to 0 because it's not used

Header header
float64[4] coefficients
geometry_msgs/Point[] points
