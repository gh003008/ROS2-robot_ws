import math
from enum import Enum

import numpy as np

import rclpy
from rclpy.node import Node

# from std_msgs.msg import String, Float32, Int64
# from msg_srv_action_interface.msg import RVD
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose
from rclpy.parameter import Parameter

class Config(Node): # 노드 생성

    def __init__(self):
        super().__init__('dwa') # 노드 이름: dwa

        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 60.0 * math.pi / 180.0  # [rad/s] Z축을 기준으로한 회전
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        #self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 1.0  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check

        self.twist = Twist() ##############
        self.linear = Vector3()
        self.angular = Vector3()

        # x = [x좌표, y좌표, angle, linear velocity, angular velocity]
        # u = [linear velocity, angular velocity]
        self.x = np.array([0,0,0,0,0])
        self.u = np.array([0,0])
        self.ob = np.array([[0.0, 0.0]])

        # waypoints 를 파라미터로 생성
        self.declare_parameter('num_waypoints', 4)
        self.num_waypoints = self.get_parameter('num_waypoints').value

        self.declare_parameter('waypoint_1', None)
        waypoint1 = self.get_parameter('waypoint_1').value

        self.declare_parameter('waypoint_2', None)
        waypoint2 = self.get_parameter('waypoint_2').value

        self.declare_parameter('waypoint_3', None)
        waypoint3 = self.get_parameter('waypoint_3').value

        self.declare_parameter('waypoint_4', None)
        waypoint4 = self.get_parameter('waypoint_4').value

        self.waypoints = np.array([waypoint1, waypoint2, waypoint3, waypoint4])
        self.i = 0
        self.goal = self.waypoints[self.i]

#### 퍼블리셔, 서브스크라이버
        self.subscriber_1 = self.create_subscription(Pose, '/turtle1/pose', self.subscribeCallbackOne, 10)

        self.subscriber_2 = self.create_subscription(Pose, '/turtle2/pose', self.subscribeCallbackTwo, 10)

        self.publisher_2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)


        #self.timer = self.create_timer(1, self.publishCallback)
        self.timer = self.create_timer(0.1, self.publishCallback)

#### 아래는 publish, subscribe 콜백함수 메서드

    def subscribeCallbackOne(self, msg): # turtle1/pose를 받음

        self.ob = np.array([[msg.x, msg.y]]) # 받아서 장애물의 위치로 저장

    def subscribeCallbackTwo(self, msg): # turtle2/pose를 받음

        # 받아서 터틀2의 위치 정보로 사용
        self.x = np.array([msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity])

    def publishCallback(self): # turtle2/cmd_vel을 뿌림

        self.linear.x = float(self.u[0])
        self.linear.y = 0.0
        self.linear.z = 0.0

        self.angular.x = 0.0
        self.angular.y = 0.0
        self.angular.z = float(self.u[1])

        self.twist.linear = self.linear
        self.twist.angular = self.angular

        trajectory = np.array(self.x)
        ob = self.ob
        self.u, predicted_trajectory = self.dwa_control(self.x, self.goal, ob)
        self.x = self.motion(self.x, self.u, self.dt)  # simulate robot
        trajectory = np.vstack((trajectory, self.x))  # store state history
        dist_to_goal = math.hypot(self.x[0] - self.waypoints[self.i][0], self.x[1] - self.waypoints[self.i][1])

        self.publisher_2.publish(self.twist) # 퍼블리시 실행
        if dist_to_goal <= self.robot_radius: # waypoint에 도달하면 랜덤으로 다음 waypoint에 가도록
            # print("Goal!!")
            self.i += 1
            # self.goal = self.waypoints[self.i]
        if self.i==3:
            self.i = 0


    ####이 아래는 dwa 알고리즘의 kinematics를 계산해주는 메서드들 이므로 원래 소스코드부분에서 그냥 가져온다.

    def dwa_control(self, x, goal, ob):

        dw = self.calc_dynamic_window(x)

        u, trajectory = self.calc_control_and_trajectory(x, dw, goal, ob)

        return u, trajectory


    def motion(self, x, u, dt):
        """
        motion model
        """

        # x = [x좌표, y좌표, angle, linear velocity, angular velocity] ; 상태
        # u = [linear velocity, angular velocity]

        x[2] += u[1] * dt # angle
        x[0] += u[0] * math.cos(x[2]) * dt # x좌표
        x[1] += u[0] * math.sin(x[2]) * dt # y좌표
        x[3] = u[0] # linear velocity
        x[4] = u[1] # angular velocity

        return x


    def calc_dynamic_window(self, x):
        """
        calculation dynamic window based on current state x
        """

        # Dynamic window from robot specification
        Vs = [self.min_speed, self.max_speed,
              -self.max_yaw_rate, self.max_yaw_rate] # window ; 선속도와 각속도의 범위를 표현한 창

        # Dynamic window from motion model
        Vd = [x[3] - self.max_accel * self.dt,
              x[3] + self.max_accel * self.dt,
              x[4] - self.max_delta_yaw_rate * self.dt,
              x[4] + self.max_delta_yaw_rate * self.dt] # 모델링한 모델의 선속도, 각속도 정보

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])] # 앞의 window(범위)의 비교를 통해 다이내믹 윈도우 저장

        # dw = [최소 선속도, 최대 선속도, 최소 각속도, 최대 각속도]

        return dw


    def predict_trajectory(self, x_init, v, y):
        """
        predict trajectory with an input
        """

        x = np.array(x_init) # 처음 상태 초기화
        trajectory = np.array(x)
        time = 0

        while time <= self.predict_time:
            x = self.motion(x, [v, y], self.dt)
            trajectory = np.vstack((trajectory, x)) # x 정보 리스트를 계속 쌓아 나가서 경로를 만듦
            time += self.dt # self 클래스에 저장한 time step

        return trajectory


    def calc_control_and_trajectory(self, x, dw, goal, ob):
        """
        calculation final input with dynamic window
        """

        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for y in np.arange(dw[2], dw[3], self.yaw_rate_resolution):

                trajectory = self.predict_trajectory(x_init, v, y)
                # calc cost
                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal)
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])
                ob_cost = self.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, ob)

                final_cost = to_goal_cost + speed_cost + ob_cost

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    if abs(best_u[0]) < self.robot_stuck_flag_cons \
                            and abs(x[3]) < self.robot_stuck_flag_cons:
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_u[1] = -self.max_delta_yaw_rate
        return best_u, best_trajectory


    def calc_obstacle_cost(self, trajectory, ob):
        """
        calc obstacle cost inf: collision
        """
        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy) # 경로(터틀2) 에서 장애물 까지의 거리

        #if self.robot_type == RobotType.rectangle:
        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= self.robot_length / 2
        right_check = local_ob[:, 1] <= self.robot_width / 2
        bottom_check = local_ob[:, 0] >= -self.robot_length / 2
        left_check = local_ob[:, 1] >= -self.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                            np.logical_and(bottom_check, left_check))).any():
            return float("Inf")
        #elif self.robot_type == RobotType.circle:
            #if np.array(r <= self.robot_radius).any():
                #return float("Inf")

        min_r = np.min(r)
        return 1.0 / min_r  # OK


    def calc_to_goal_cost(self, trajectory, goal):
        """
            calc to goal cost with angle difference
        """

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost


def main(args=None):
    rclpy.init(args=args)

    node = Config()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrypt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
