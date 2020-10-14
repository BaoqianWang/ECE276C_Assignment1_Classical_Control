import gym
import pybulletgym.envs
import numpy as np
import matplotlib.pyplot as plt
import random
random.seed(30)
np.random.seed(30)

class DoF2Arm():
    def __init__(self, l0, l1):
        self.env = gym.make("ReacherPyBulletEnv-v0")
        self.env.reset()
        self.set_q0(0, 0)
        self.set_q1(0, 0)
        self.l0 = l0
        self.l1 = l1
        self.Kp = 0.94
        self.Kd = .002
        # self.Kp = 2.24
        # self.Kd = .0009
        self.dt = 0.01

        # Initial angles of two joints
        #self.q0 = 0
        #self.q1 = 0

    def getForwardModel(self, q0, q1):
        angle_matrix = np.array([[np.cos(q0), np.cos(q0+q1)],[np.sin(q0), np.sin(q0+q1)]])
        p = angle_matrix.dot(np.array([[self.l0],[self.l1]]))
        # print(p)
        return p[0], p[1]

    def getJacobian(self, q0, q1):
        J = np.zeros((2,2))
        J[0][0] = -self.l0*np.sin(q0)-self.l1*np.sin(q0+q1)
        J[0][1] = -self.l1*np.sin(q0+q1)
        J[1][0] =  self.l0*np.cos(q0)+self.l1*np.cos(q0+q1)
        J[1][1] =  self.l1*np.cos(q1+q0)

        return J


    def pd_controller(self):
        desired_x, desired_y = self.desired_trajectory()
        real_x_list = []
        real_y_list = []
        errorx_prev = 0
        errory_prev = 0
        for i in range(len(desired_x)):
            q0, q0_dot = self.get_q0()
            q1, q1_dot = self.get_q1()
            realx, realy = self.getForwardModel(q0, q1)

            real_x_list.append(realx)
            real_y_list.append(realy)

            J = self.getJacobian(q0, q1)

            errorx = desired_x[i] - realx[0]
            errory = desired_y[i] - realy[0]

            errordx = (errorx_prev - errorx)/self.dt
            errordy = (errory_prev - errory)/self.dt

            errorx_prev = errorx
            errory_prev = errory

            x_force  = self.Kp * errorx + self.Kd * errordx
            y_force  = self.Kp * errory + self.Kd * errordy



            torque = J.T.dot(np.array([[x_force], [y_force]]))

            action = [torque[0], torque[1]]
            self.env.step(action)

            # delta_p = np.array([[errorx],[errory]])
            # delta_q = np.linalg.pinv(J).dot(delta_p)
            # self.set_q0(q0+delta_q[0],0)
            # self.set_q1(q1+delta_q[1],0)
        error = [np.sqrt((real_x_list[i]-desired_x[i])**2 + (real_y_list[i]-desired_y[i])**2) for i in range(len(real_x_list))]
        time_traj = np.linspace(0,10,1000)
        plt.figure(1)
        plt.plot(real_x_list, real_y_list, label='Real')
        plt.plot(desired_x, desired_y, label='Desired')
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.legend()
        plt.show(block=True)

        plt.figure(2)
        plt.plot(time_traj, real_x_list, label='Real')
        plt.plot(time_traj, desired_x, label='Desired')

        plt.xlabel('Time (s)')
        plt.ylabel('x (m)')
        plt.legend()
        plt.show(block=True)

        plt.figure(3)
        plt.plot(time_traj, real_y_list, label='Real')
        plt.plot(time_traj, desired_y, label='Desired')

        plt.xlabel('Time (s)')
        plt.ylabel('y (m)')
        plt.legend()
        plt.show(block=True)

        plt.figure(4)
        plt.plot(time_traj, error)
        plt.xlabel('Time (s)')
        plt.ylabel('Root squared error (m)')
        plt.show(block=True)
        plt.legend()

        return



    # def pd_controller(self):
    #     desired_x, desired_y = self.desired_trajectory()
    #     real_x_list = []
    #     real_y_list = []
    #     errorx_prev = 0
    #     errory_prev = 0
    #     for i in range(len(desired_x)):
    #         q0, q0_dot = self.get_q0()
    #         q1, q1_dot = self.get_q1()
    #         realx, realy = self.getForwardModel(q0, q1)
    #
    #         real_x_list.append(realx)
    #         real_y_list.append(realy)
    #
    #         J = self.getJacobian(q0, q1)
    #
    #         errorx = desired_x[i] - realx[0]
    #         errory = desired_y[i] - realy[0]
    #
    #         errordx = (errorx_prev - errorx)/self.dt
    #         errordy = (errory_prev - errory)/self.dt
    #
    #         errorx_prev = errorx
    #         errory_prev = errory
    #
    #         x_force  = self.Kp * errorx + self.Kd * errordx
    #         y_force  = self.Kp * errory + self.Kd * errordy
    #         action = [x_force, y_force]
    #         self.env.step(action)
    #         #torque = J.T.dot(np.array([[], []]))
    #         # delta_p = np.array([[errorx],[errory]])
    #         # delta_q = np.linalg.pinv(J).dot(delta_p)
    #         # self.set_q0(q0+delta_q[0],0)
    #         # self.set_q1(q1+delta_q[1],0)
    #
    #     plt.figure(1)
    #     plt.plot(real_x_list, real_y_list)
    #     plt.plot(desired_x, desired_y)
    #     plt.show(block=True)
    #
    #     plt.figure(2)
    #     plt.plot(desired_x)
    #     plt.plot(real_x_list)
    #     plt.show(block=True)
    #
    #     plt.figure(3)
    #     plt.plot(desired_y)
    #     plt.plot(real_y_list)
    #     plt.show(block=True)
    #
    #     return


    # def pd_controller(self):
    #     desired_x, desired_y = self.desired_trajectory()
    #     real_x_list = []
    #     real_y_list = []
    #     for i in range(len(desired_x)):
    #         q0, q0_dot = self.get_q0()
    #         q1, q1_dot = self.get_q1()
    #         realx, realy = self.getForwardModel(q0, q1)
    #
    #         real_x_list.append(realx)
    #         real_y_list.append(realy)
    #
    #         J = self.getJacobian(q0, q1)
    #
    #         errorx = desired_x[i] - realx[0]
    #         errory = desired_y[i] - realy[0]
    #         delta_p = np.array([[errorx],[errory]])
    #         delta_q = np.linalg.pinv(J).dot(delta_p)
    #         self.set_q0(q0+delta_q[0],0)
    #         self.set_q1(q1+delta_q[1],0)
    #
    #     plt.figure(1)
    #     plt.plot(real_x_list, real_y_list)
    #     plt.plot(desired_x, desired_y)
    #     plt.show(block=True)
    #
    #     plt.figure(2)
    #     plt.plot(desired_x)
    #     plt.plot(real_x_list)
    #     plt.show(block=True)
    #
    #     plt.figure(3)
    #     plt.plot(desired_y)
    #     plt.plot(real_y_list)
    #     plt.show(block=True)
    #
    #     return


    def desired_trajectory(self):
        theta = np.linspace(0, 2*np.pi, 1000)
        #print(theta)
        x = (0.19 + 0.02*np.cos(4*theta))*np.cos(theta)
        y = (0.19 + 0.02*np.cos(4*theta))*np.sin(theta)

        return x, y

    def set_q0(self, angle, angle_dot):
        self.env.unwrapped.robot.central_joint.reset_position(angle, angle_dot)

        #return q0, q0_dot

    def get_q0(self):
        q0, q0_dot = self.env.unwrapped.robot.central_joint.current_position()
        return q0, q0_dot

    def set_q1(self, angle, angle_dot):
        self.env.unwrapped.robot.elbow_joint.reset_position(angle, angle_dot)

    def get_q1(self):
        q1, q1_dot = self.env.unwrapped.robot.elbow_joint.current_position()
        return q1, q1_dot



if __name__ == '__main__':
    dof2arm_instance = DoF2Arm(0.1, 0.11)
    print(dof2arm_instance.getForwardModel(0,0))
    print(dof2arm_instance.getJacobian(0,0))
    x, y = dof2arm_instance.desired_trajectory()
    #print(x,y)
    dof2arm_instance.pd_controller()
    # print(dof2arm_instance.set_q0(0.3, 0))
    # plt.figure()
    # plt.plot(x,y)
    # plt.show(block=True)
