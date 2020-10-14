# import gym
# import pybulletgym.envs
import numpy as np
import matplotlib.pyplot as plt
import random
from racecar.SDRaceCar import SDRaceCar
import matplotlib.pyplot as plt
from math import *
random.seed(30)
np.random.seed(30)


class Car():
    def __init__(self):
        self.env = SDRaceCar(render_env = True, track = 'FigureEight')
        self.env.reset()
        self.Kp = 10
        self.Kd = 10
        self.Kthetap = -1
        self.Kthetad = -.5
        return


    def pd_controller(self):
        #self.env = SDRaceCar(render_env = True, track = 'FigureEight')
        color_set = []
        for i in range(2):
            color_set.append(np.random.rand(3))

        # fig = plt.figure()
        # plt.axis([-100, 100, -100, 100])
        # ax = fig.add_subplot(1, 1, 1)
        # plt.xlabel('X (m)')
        # plt.ylabel('Y (m)')
        done = False
        x_list = []
        y_list = []
        xd_list = []
        yd_list = []
        while(not done):
            s = self.env.get_observation()
            #self.env.render()
            x_ref = s[-1][0]
            y_ref = s[-1][1]
            #dx_ref = 0
            #dy_ref = 0
            real_x = s[0]
            real_y = s[1]
            x_list.append(real_x)
            y_list.append(real_y)
            xd_list.append(x_ref)
            yd_list.append(y_ref)
            #real_vx = s[3]
            #real_vy = s[4]
            #theta_ref =  np.arctan((y_ref - real_y)/(x_ref - real_x))
            theta_ref = atan2(y_ref - real_y, x_ref - real_x)
            theta = s[2]
            theta_dot =s[5]
            #print('Angle', s[2], theta_ref)
            # circ1 = plt.Circle((real_x, real_y), radius=0.3, fc='w', ec=color_set[0])
            # ax.add_patch(circ1)
            #
            # circ2= plt.Circle((x_ref, y_ref), radius=0.3, fc='w', ec=color_set[1])
            # ax.add_patch(circ2)
            #
            # plt.draw()
            # plt.pause(0.01)
            if (theta_ref - theta > np.pi):
                theta += 2*np.pi

            if (theta - theta_ref > np.pi):
                theta_ref += 2*np.pi

            u_x = self.Kp*(theta_ref - theta) + self.Kd*(0 - theta_dot)

            if (np.abs(theta_ref - theta) < np.pi/10):
                u_y = 0.05
            else:
                u_y = self.Kthetap *(np.abs(theta_ref - theta)) + self.Kthetad*(np.abs(0 - theta_dot))
            #u_y = self.Kp*(y_ref - real_y) + self.Kd*(dy_ref - real_vy)
            #print('Control', u_x, u_y)
            action= [u_x, u_y]
            obs, reward, done = self.env.step(action)
            print(done)
        plt.figure(1)
        plt.plot(x_list, y_list, label='Real')
        plt.plot(xd_list, yd_list, label='Desired')
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.legend()
        plt.show()
        plt.waitforbuttonpress()

        return







if __name__ == '__main__':
    car  = Car()
    #s = car.env.get_observation()
    car.pd_controller()
    #print(s[-1][0])
    #print(dof2arm_instance.getJacobian(0,0))

    #print(x,y)
    #dof2arm_instance.pd_controller()
