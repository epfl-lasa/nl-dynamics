
README file for GPMDS matlab code 

Run the function interactiveGPreshape. It will load a figure showing trajectories of a 2d diagonal linear system with an attractive equilibrium point at the origin. The figure is set up so that the dynamics can be interactively updated by "drawing" demonstrations on the figure. 

(Right-click-and-drag) in the figure will draw a demonstration. The positional data is passed to a kalman filter that will render smooth and consistent demonstration trajectories of position and velocity. This data is then passed to the GP-MDS algorithm which uses it to reshape the original dynamics so that it will 1) any trajectory starting on the demonstrated path will stay there and 2) it will provide in a sense a generalization of the demonstration, where nearby paths tend to follow the demonstrated path. The figure is updated to show the streamlines of the updated system. In addition, several new objects are plotted:

1) red dots indicating the demonstrated position trajectory
2) green circles indicating which of the demonstrated points are used by the GP
3) a gradient color map showing the region of influence of the GP


(left-click) start a simulation of a trajectory. A trajectory started from the location of the mouse at the time of clicking will be integrated its progression in the task space is shown. 

(middle-click) Reinitialize the system (equivalent to re-running the function)


The system can lose its asymptotic stability if the origin is reshaped. Also, if limit cycle behavior is demonstrated, the most likely outcome is a half-stable limit cycle, where trajectories starting close to the origin converge to the origin but trajectories starting outside the demonstrated limit cycle converge to the limit cycle. Also note that as per Bendixons criterion, a limit cycle by necessity encircles an equilibrium point and is hence impossible to achieve unless the demonstrated cycle encircles the origin. Cyclic demonstrations that do not encircle the origin result in erratic (but asymptotically stable) dynamics. 

In the interactiveGPreshape functions, some parameters are marked as %INTERESTING PARAMETER TO PLAY WITH. These include the kernel width and noise levels for the GP, which obviously have a significant impact on how the system reacts to new demonstrations. 



