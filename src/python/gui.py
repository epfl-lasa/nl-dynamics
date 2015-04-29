#!/usr/bin/env python
NODE_NAME = 'gui'

import roslib; roslib.load_manifest(NODE_NAME)
import rospy

import sys
import time
import os

# Messages
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from dyn_sys.msg import *
from geometry_msgs.msg import PoseStamped
# Services
from dyn_sys.srv import * 

from numpy import *
import matplotlib
matplotlib.use('GTK3Agg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.widgets import CheckButtons
from matplotlib.animation import FuncAnimation
from matplotlib import gridspec
from matplotlib.patches import Rectangle



class Gui():
    def __init__(self, fig, ax):
        rospy.init_node(NODE_NAME, anonymous=True)
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo('Starting up gui node.')

        self.current_pose = None
        self.current_poseSim = None

        self.current_pose_changed = False
        self.current_pose_init = False


        # Subscribers
        rospy.Subscriber('/KUKA/Pose', PoseStamped, self.update_pose)
        rospy.Subscriber('/KUKA/PoseSim', Position, self.update_poseSim)
        rospy.Subscriber('/demonstration', Demonstration, self.update_demos)

        # Publishers
        self.user_mouse = rospy.Publisher('/mouse_info', Position)
        self.switch_pubK = rospy.Publisher('/demo_switchK', Bool)
        self.switch_pubG = rospy.Publisher('/demo_switchG', Bool)
        self.startTrial_pub = rospy.Publisher('/start_trial', Bool)
        self.traj_pub = rospy.Publisher('/referenceTrajectory', Trajectory)
        self.trajS_pub = rospy.Publisher('/referenceTrajectorySim', Trajectory)
        self.quit_pub = rospy.Publisher('/user_study/quit', Bool)
        
        # Services
        rospy.wait_for_service('get_updated_dynamics')
        self.get_updated_dynamics = rospy.ServiceProxy('get_updated_dynamics', GetUpdatedDynamics)

        rospy.wait_for_service('reset_dynamics')
        self.reset_dynamics = rospy.ServiceProxy('reset_dynamics', ResetDynamics)

        rospy.wait_for_service('remove_demo')
        self.remove_demo = rospy.ServiceProxy('remove_demo', RemoveDemo)

        rospy.wait_for_service('get_trajectory')
        self.get_trajectory = rospy.ServiceProxy('get_trajectory', GetTrajectory)


        # GUI Variables
        self.background = None
        self.backgrounds = rospy.get_param('background_img')


        self.plot_range = rospy.get_param('plot_range')
        self.nX = rospy.get_param('nX')    

        [self.xM, self.yM] = meshgrid(linspace(self.plot_range[0], self.plot_range[1], self.nX), linspace(self.plot_range[2], self.plot_range[3], self.nX))
        X = concatenate((transpose(reshape(self.xM, (size(self.xM),1), order='F')),transpose(reshape(self.yM, (size(self.yM),1), order='F'))),axis=0)

        self.plot_pos = []
        for i in range(0, size(X[0])):
            self.plot_pos.append(Position([X[0,i],X[1,i]]))
        
        self.U, self.V = self.get_velocities(self.plot_pos, self.nX)
        
        self.demos = []
        self.drawing_demo = False
        self.moving_demo = False
        self.need_replot = False

        # Plots for the current robot position and the demos
        self.pose_plot = None
        self.pose_plotSim = None

        self.demo_plot = []


        self.trial = None
        self.demo = None

        self.target =  rospy.get_param('target')
        self.starts =  rospy.get_param('start_locations')
        self.obstacles = rospy.get_param('obstacle_locations') 

        self.stream_color = '#606060'
        self.init_gui(fig, ax)

        self.trial_num = 0
        self.trial_mode = -1

        with open('user_num.txt', 'r') as f:
            self.user_num = int(f.read())
        with open('user_num.txt', 'w') as f:
            f.write(str(self.user_num  +1))

        #self.play_trajs = []


    def init_gui(self, fig, ax):
        # Initialize GUI
        self.fig = fig
        self.ax = ax

        self.ax.set_xlim([self.plot_range[0],self.plot_range[1]])
        self.ax.set_ylim([self.plot_range[2],self.plot_range[3]])

        self.draw_background()


        self.sp = self.ax.streamplot(self.xM, self.yM, self.U, self.V, density = 1.0, color = self.stream_color)

        # Initialize mouse events
        self.mouseR_press = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.mouseR_release = self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.mouse_move = self.fig.canvas.mpl_connect('motion_notify_event', self.mouse_move)

        # Initialize buttons
        button_hpos = 0.775
        button_hsize = 0.2
        
        button_vsize = 0.060


        self.ax_kin = plt.axes([0.060, 0.87, button_hsize, button_vsize])#([button_hpos, 0.9, button_hsize, button_vsize])
        self.bt_kin = Button(self.ax_kin, 'Start Kin Demo', color= '0.85', hovercolor= '0.85')
        self.bt_kin.on_clicked(self.switch_kin)

        self.ax_straj = plt.axes([0.360, 0.9, button_hsize, button_vsize])#([button_hpos, 0.8, button_hsize, button_vsize])
        self.bt_straj = Button(self.ax_straj, 'Play Robot \n Trajectories', color= '0.85', hovercolor= '0.85')
        self.bt_straj.on_clicked(self.start_traj)

        self.ax_strajS = plt.axes([0.360, 0.83, button_hsize, button_vsize])#([button_hpos, 0.8, button_hsize, button_vsize])
        self.bt_strajS = Button(self.ax_strajS, 'Play Sim \n Trajectories', color= '0.85', hovercolor= '0.85')
        self.bt_strajS.on_clicked(self.start_trajS)


        self.ax_undo = plt.axes([0.660, 0.87, button_hsize, button_vsize])#([button_hpos, 0.7, button_hsize, button_vsize])
        self.bt_undo = Button(self.ax_undo, 'Undo Last Demo', color= '0.85', hovercolor= '0.85')
        self.bt_undo.on_clicked(self.undo)


        self.ax_ctrial = plt.axes([0.060, 0.01, button_hsize, button_vsize*1.5])#plt.axes([button_hpos, 0.375, button_hsize, button_vsize*1.5])
        self.check_trial = CheckButtons(self.ax_ctrial, ('Training', 'Trial 1', 'Trial 2', 'Trial 3'), (False, False, False, False))
        self.check_trial.on_clicked(self.change_trial)

        self.ax_strial = plt.axes([0.060, 0.1, button_hsize, button_vsize])#plt.axes([button_hpos, 0.3, button_hsize, button_vsize])
        self.bt_strial = Button(self.ax_strial, 'Start Trial', color= '0.85', hovercolor= '0.85')
        self.bt_strial.on_clicked(self.start_trial)

        self.ax_reset = plt.axes([0.360, 0.1, button_hsize, button_vsize])#plt.axes([button_hpos, 0.2, button_hsize, button_vsize])
        self.bt_reset = Button(self.ax_reset, 'Reset System', color= '0.85', hovercolor= '0.85')
        self.bt_reset.on_clicked(self.reset)

        self.ax_quit = plt.axes([0.660, 0.1, button_hsize, button_vsize])#plt.axes([button_hpos, 0.1, button_hsize, button_vsize])
        self.bt_quit = Button(self.ax_quit, 'Quit', color= '0.85', hovercolor= '0.85')
        self.bt_quit.on_clicked(self.quit)
        
        plt.show(block=False)

    def change_trial(self,event):
        print 'Changing trial mode to ' + event
        if self.trial_mode == -1:
            print 'Switching trial.'
            if event == 'Training':
                self.trial_mode = 0
            elif event == 'Trial 1':
                self.trial_mode = 1
            elif event == 'Trial 2':
                self.trial_mode = 2
            else:
                self.trial_mode = 3
        else:
            self.trial_mode = -1

    def draw_background(self):
        if self.trial != None:
            img = plt.imread('../backgrounds/' + self.background)
            ax.imshow(img, zorder=0, extent=[-1.00, -0.17, -0.472, 0.735])
        else:
            img = plt.imread('../backgrounds/startscene.jpg')
            ax.imshow(img, zorder=0, extent=[-1.00, -0.17, -0.472, 0.735])

            #for s in self.start:
            #    self.ax.scatter([s[0]], [s[1]], color='y', marker="o", s=300)

            #for o in self.obstacle:
            #    self.ax.scatter([o[0]], [o[1]], color='g', marker="o", s=300)



    def get_velocities(self, pos, nX):
        try:
            rospy.loginfo('Trying to call service.')
            resp = self.get_updated_dynamics(pos)
            rospy.loginfo('Service call successful.')
            Xd = zeros((2,size(resp.velocities)))
            for i in range(0, size(resp.velocities)):
                Xd[:,i] = resp.velocities[i].position
            U = reshape(Xd[0,:], (nX,nX), order='F')
            V = reshape(Xd[1,:], (nX,nX), order='F')
            return U, V
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return None, None

      
    def on_shutdown(self):
        rospy.loginfo('GUI is quitting.')

    # Subscriber callbacks
    def update_pose(self, p):
        if self.current_pose == None and self.current_pose_init == False:
            self.current_pose = p.pose
            self.current_pose_init = True
            self.current_pose_changed = True
        if absolute(self.current_pose.position.x - p.pose.position.x) > 0.001 or absolute(self.current_pose.position.y - p.pose.position.y) > 0.001:
            self.current_pose_changed = True#self.current_pose = p.pose
            self.current_pose = p.pose
        else:
            self.current_pose_changed = False
        #else:
        #    self.current_pose = None
            #print 'pose not changed.'

    def update_poseSim(self, p):
        self.current_poseSim = p
        self.pose_plotSim_changed  = True

    def update_demos(self, d):
        self.need_replot = True
        if d.type:
            c = '#9900CC'
        else:
            c = 'r'
        g = []
        for p in d.positions:
            s = ax.scatter([p.position[0]], [p.position[1]], color=c, marker="o")
            g.append(s)
            if self.demo != None:
                self.demo.data.append(p.position)
        if self.trial != None and self.demo!=None:
            self.trial.add_demo(self.demo)#self.trial.demos.append(self.demo)
            self.demo = None

        self.demo_plot.append(g)
        self.demos.append(d)
        time.sleep(0.1)

    # Mouse Events
    def on_click(self, event):
        #if not event.inaxes: return
        if not event.inaxes or event.inaxes == self.ax_reset or event.inaxes == self.ax_kin or event.inaxes == self.ax_quit:
            return
        if event.button == 3:
            # Error- demo is already ongoing
            if self.drawing_demo == True:
                self.drawing_demo = False
            if event.xdata != None and event.ydata != None:
                self.drawing_demo = True
            self.switch_pubG.publish(self.drawing_demo)
            self.demo = Demo(time.time(),False)


    def on_release(self, event):
        #if not event.inaxes: return
        if not event.inaxes or event.inaxes == self.ax_reset or event.inaxes == self.ax_kin or event.inaxes == self.ax_quit:
            return
        if event.button == 3:
            if self.drawing_demo == True:
                self.drawing_demo = False
                self.switch_pubG.publish(self.drawing_demo)
                self.demo.end_time = time.time()

    def mouse_move(self, event):
        # If the mouse is moving outside of the GUI
        if not event.inaxes: return
        self.user_mouse.publish(Position([event.xdata,event.ydata]))# = rospy.Publisher('/mouse_info', Position)


    # Button Events
    def quit(self, event):
        self.quit_pub.publish(True)
        if self.trial!= None:
            self.trial.save_trial()
        sys.exit(0)

    def reset(self, event):
        self.reset_system()

    def reset_system(self):
        try:
            rospy.loginfo('Trying to reset dynamics and gui.')
            resp = self.reset_dynamics()
            if resp.success == True:
                self.drawing_demo = False
                # reset variables
                for d in self.demo_plot:
                    for s in d:
                        s.remove()
                self.demo_plot = []
                self.demos = []
                self.need_replot = True
            rospy.loginfo('Reset successful.')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def switch_kin(self, event):
        # change button label
        if self.moving_demo == True:
            self.moving_demo = False
            self.bt_kin.label.set_text('Start Kin Demo')
            # End demo
            self.demo.end_time = time.time()
        else:
            self.moving_demo = True
            self.bt_kin.label.set_text('Stop Kin Demo')
            self.demo = Demo(time.time(),True)
        self.switch_pubK.publish(self.moving_demo) 
        self.fig.canvas.draw()

    def undo(self, event):
        rospy.loginfo('Removing last demo.')
        if len(self.demo_plot) == 0:
            rospy.loginfo('No demo to undo.')
            return
        # remove last demo from dyn_sys
        resp = self.remove_demo(len(self.demo_plot)-1)
        if resp.success == True:#self.remove_pub.publish(len(self.demo_plot)-1)
            # remove from plot
            self.demos.pop()
            d = self.demo_plot.pop()
            for s in d:
                s.remove()
            # replot
            self.need_replot = True
            if self.trial!=None:
                self.trial.undo()
        #print self.need_replot

    def start_traj(self, event):
        rospy.loginfo('Playing trajectories.')
        if self.trial == None:
            rospy.loginfo('Error. No trial so no trajectories to play.')
            return
        for i in range(0,len(self.trial.start_points)):
            start = time.time()
            resp = self.get_trajectory(Position(self.trial.start_points[i]))
            print 'Trajectory ' + str(i) + ' took ' + str(time.time()-start) + ' seconds.'
            self.traj_pub.publish(Trajectory(resp.trajectory))

    def start_trajS(self, event):
        rospy.loginfo('Playing trajectories.')
        if self.trial == None:
            rospy.loginfo('Error. No trial so no trajectories to play.')
            return
        trajs = []
        for i in range(0,len(self.trial.start_points)):
            start = time.time()
            resp = self.get_trajectory(Position(self.trial.start_points[i]))
            print 'Trajectory ' + str(i) + ' took ' + str(time.time()-start) + ' seconds.'
            trajs.append(resp.trajectory)
        for t in trajs:
            self.trajS_pub.publish(t)#resp.trajectory)
            #self.play_trajs.append(resp.trajectory)
            

    def start_trial(self, event):
        print 'Pressed trial button.'
        print self.trial == None
        if self.trial == None:
            print self.trial_mode
            if self.trial_mode != -1:
                rospy.loginfo('Starting trial ' + str(self.trial_mode))
                self.start = self.starts[self.trial_mode]
                self.background = self.backgrounds[self.trial_mode]
                self.trial = Trial(self.start, self.trial_mode, self.user_num, self.trial_num)
                self.draw_background()
                self.bt_strial.label.set_text('Stop Trial')
        else:
            self.trial.save_trial()
            self.trial_num = self.trial_num + 1
            self.trial = None
            self.start = None
            self.background = None
            self.trial_mode = -1
            self.reset_system()
            self.draw_background()
            self.bt_strial.label.set_text('Start Trial')
        self.fig.canvas.draw()

    # Draw a trajectory in simulation
    def run_simTraj(self, traj):
        start = time.time()
        play_pose_plot = None
        for i in range(0,len(traj),100):
            if play_pose_plot != None:
                play_pose_plot.remove()
            p = traj[i]
            play_pose_plot = self.ax.scatter([p.position[0]],[p.position[1]], color='#FFFF00', marker='o', s=(3.14*pow(13,2)),zorder=1)
            self.fig.canvas.draw()
        play_pose_plot.remove()
        self.fig.canvas.draw()
        rospy.loginfo('Drawing sim trajectory took ' + str(time.time()-start) + ' seconds.')        

    # Draw the robot's current pose
    def draw_robotPos(self):
        if self.current_pose != None and self.pose_plot == None:
            self.pose_plot = self.ax.scatter([self.current_pose.position.x],[self.current_pose.position.y], color='#ff9900', marker='o', s=(3.14*pow(13,2)),zorder=1)
            self.fig.canvas.draw()

        if self.current_pose != None and self.current_pose_changed:
            if self.pose_plot != None:
                self.pose_plot.remove()
            self.pose_plot = self.ax.scatter([self.current_pose.position.x],[self.current_pose.position.y], color='#ff9900', marker='o', s=(3.14*pow(13,2)),zorder=1)
            self.fig.canvas.draw()
            #self.current_pose = None

        # Play any simulation trajectories
        if self.current_poseSim != None:
            if self.pose_plotSim != None:
                self.pose_plotSim.remove()
            self.pose_plotSim = self.ax.scatter([self.current_poseSim.position[0]],[self.current_poseSim.position[1]], color='#FFFF00', marker='o', s=(3.14*pow(13,2)),zorder=1)
            self.fig.canvas.draw()
            self.current_poseSim = None


    # Animation Call
    def __call__(self, i):
        # Draw robot position
        self.draw_robotPos()


        #if len(self.play_trajs) > 0:
        #    traj = self.play_trajs.pop(0)
        #    self.run_simTraj(traj)

        # Replot streamlines
        if self.need_replot:
            # Remove lines and arrows
            if self.sp!=None:
                if self.sp.lines!=None:
                    self.sp.lines.remove()
                    rospy.sleep(0.1)
                    self.ax.patches = []
                    self.sp = None
            # Update velocities
            self.draw_background()
            rospy.loginfo('Getting new velocities.')
            self.U, self.V = self.get_velocities(self.plot_pos, self.nX)
            rospy.loginfo('Replotting gui.')
            # Replot streamlines
            self.sp = self.ax.streamplot(self.xM, self.yM, self.U, self.V, density = 1.0, color = self.stream_color)
            rospy.loginfo('Done with plot lines')
            self.need_replot = False
            
            self.fig.canvas.draw()

class Demo():
    def __init__(self, start_time, demo_type):
        self.start_time = start_time
        self.end_time  = None#end_time
        self.demo_type = demo_type
        self.data = []#data

class Trial():
    def __init__(self, start_points, scene_num, user_num, trial_num):
        self.start_points = start_points
        self.scene_num = scene_num
        self.user_num = user_num
        self.trial_num = trial_num
        self.demos = []
        self.kept = []

    def add_demo(self, d):
        self.demos.append(d)
        self.kept.append(1)

    def undo(self):
        # Search from back through demo and find last kept demo
        for i in range(len(self.kept)-1,0-1,-1):
            if self.kept[i] == 1:
                self.kept[i] = 0
                return        

    def save_trial(self):
        rospy.loginfo('Saving trial.')
        filename = 'Data/User' + str(self.user_num) + '/Trial' + str(self.trial_num) + '.txt'
        if not os.path.exists(os.path.dirname(filename)):
            os.makedirs(os.path.dirname(filename))
        # write header
        with open(filename, "w") as f:
            f.write(str(self.start_points) + ',' + str(self.user_num) + ',' + str(self.scene_num) + ',' + str(self.trial_num) + '\n')
            count = 0
            for d in self.demos:
                f.write(str(d.start_time) + ',' + str(d.end_time) + ',')
                if d.demo_type: f.write('1')
                else: f.write('0')
                f.write(',' + str(self.kept[count]))
                for dp in d.data:
                    f.write(',' + str(dp))
                f.write('\n')
                count = count + 1


fig = plt.figure(figsize=(12,22))#fig = plt.figure()#plt.figure(figsize=(10,10))
plt.rcParams['font.size']=20
gs = gridspec.GridSpec(3,1, height_ratios=[1,16,1])#gridspec.GridSpec(1, 2, width_ratios=[8,1])
ax = plt.subplot(gs[1])
#ax = plt.subplot(121)
#fig, ax = plt.subplots()
gui = Gui(fig, ax)
anim = FuncAnimation(fig, gui, frames=arange(100), interval=0.1, blit=False)
plt.show()

