
These are the useful classes I took out from Liz code. The three essential components are there:

1) MultiGPR class, which is GPR for multiple outputs but identical inputs. It is verified and should be straightforward to understand. 

2) KalmanFilterFuncitons. This is Liz transaltion of my kalman stuff from matlab to python. These are also verfied, and are used the same way as the matlab counterparts. 

3) GPMDS class, which implements the algorithmic parts of the interactiveGPReshape function. 

What you need to do is create a new gui class and handle the interactivity, and interface with the GPMDS class. I also put the gui that Liz created, but you ill not be able to use it as is since she had a complex system with the GUI in a separate ros node from the gpmds etc, which was not working so well. I think it is better to rewrite the GUI, but I anyway put liz file here so you can check how to do the streamlines etc if needed. 
