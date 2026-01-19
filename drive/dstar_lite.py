import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from math import hypot, atan2, cos, sin, pi
import heapq
import numpy as np

# D* Lite 알고리즘 (격자 기반)
class DStarLite:
    def __init__(self):
        self.width = 0
        self.height = 0
        self.map = None
        self.g = {}
        self.rhs = {}
        self.U = []
        self.start = None
        self.goal = None
        self.km = 0

    def init_map(self, width, height, data):
        # OccupancyGrid 메시지를 D* Lite용 2D 맵으로 변환
        self.width = width
        self.height = height
        grid = np.array(data).reshape((height, width))
        self.map = np.where(grid > 50, 1, 0)    # 50 초과 시, 장애물로 간주
        self.g.clear()
        self.rhs.clear()
        self.U.clear()

    def heuristic(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return dx + dy

    def calculate_key(self, s):
        m = min(self.g.get(s, float('inf')), self.rhs.get(s, float('inf')))
        return [m + self.heuristic(self.start, s) + self.km, m]

    def initialize(self, start, goal):
        self.start = start
        self.goal = goal
        self.rhs[goal] = 0.0    # rhs를 0으로 설정
        heapq.heappush(self.U, self.calculate_key(goal) + list(goal))

    def neighbors(self, u):
        x, y = u
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                yield (nx, ny)

    def cost(self, u, v):
        if self.map[v[1]][v[0]] == 1:
            return float('inf')
        return 1.0

    def update_vertex(self, u):
        # rhs 값 갱신
        if u != self.goal:
            self.rhs[u] = min(
                self.g.get(s, float('inf')) + self.cost(u, s)
                for s in self.neighbors(u)
            )
        self.U = [i for i in self.U if (i[2], i[3]) != u]
        heapq.heapify(self.U)

        if self.g.get(u, float('inf')) != self.rhs.get(u, float('inf')):
            heapq.heappush(self.U, self.calculate_key(u) + list(u))

    def compute_shortest_path(self):
        # D* Lite 핵심 루프
        while self.U and (
            self.U[0][:2] < self.calculate_key(self.start) or
            self.rhs.get(self.start, float('inf')) != self.g.get(self.start, float('inf'))
        ):
            u_item = heapq.heappop(self.U)
            u = (u_item[2], u_item[3])
            if self.g.get(u, float('inf')) > self.rhs.get(u, float('inf')):
                self.g[u] = self.rhs[u]
                for s in self.neighbors(u):
                    self.update_vertex(s)
            else:
                self.g[u] = float('inf')
                self.update_vertex(u)
                for s in self.neighbors(u):
                    self.update_vertex(s)

    def extract_path(self):
        path = [self.start]
        current = self.start
        while current != self.goal:
            current = min(
                self.neighbors(current),
                key=lambda s: self.g.get(s, float('inf')),
                default=None
            )
            if current is None:
                break
            path.append(current)
        return path
    
# ROS2 노드 (D* Lite + 경로 추종)
class DStarPatrolNode(Node):
    def __init__(self):
        super().__init__('dstar_patrol')

        self.planner = DStarLite()
        self.path = []
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 구독
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        # 발행
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/dstar_path', 10)

        self.create_timer(0.1, self.control_loop)

    # 월드 좌표를 격자 좌표로 변환
    def world_to_grid(self, x, y):
        return (
            int((x - self.origin_x) / self.resolution),
            int((y - self.origin_y) / self.resolution)
        )

    # 격자 좌표를 월드 좌표로 변환
    def grid_to_world(self, x, y):
        return (
            x * self.resolution + self.origin_x,
            y * self.resolution + self.origin_y
        )

    def get_pose(self):
        # 로봇 현재 위치 (x, y, yaw) 획득
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )
            q = t.transform.rotation
            yaw = atan2(
                2*(q.w*q.z),
                1 - 2*(q.z*q.z)
            )
            return (
                t.transform.translation.x,
                t.transform.translation.y,
                yaw
            )
        except:
            return None

    # 맵 수신 시 D* Lite용 맵 초기화
    def map_cb(self, msg):
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.planner.init_map(msg.info.width, msg.info.height, msg.data)

    # 목표 지점 수신 및 경로 재계산
    def goal_cb(self, msg):
        pose = self.get_pose()
        if pose is None:
            return

        start = self.world_to_grid(pose[0], pose[1])
        goal = self.world_to_grid(
            msg.pose.position.x,
            msg.pose.position.y
        )

        self.planner.initialize(start, goal)
        self.planner.compute_shortest_path()
        self.path = self.planner.extract_path()
        self.publish_path()

    # 경로 추종
    def control_loop(self):
        if not self.path:
            return

        pose = self.get_pose()
        if pose is None:
            return

        rx, ry, ryaw = pose

        gx, gy = self.grid_to_world(*self.path[0])
        dist = hypot(gx - rx, gy - ry)

        # waypoint 도달 시 제거
        if dist < 0.2:
            self.path.pop(0)
            return

        angle = atan2(gy - ry, gx - rx)
        yaw_err = (angle - ryaw + pi) % (2*pi) - pi

        cmd = Twist()
        cmd.linear.x = 0.15
        cmd.angular.z = 0.6 * yaw_err
        self.cmd_pub.publish(cmd)

    # rviz 시각화를 위해
    def publish_path(self):
        msg = Path()
        msg.header.frame_id = 'map'
        for gx, gy in self.path:
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = self.grid_to_world(gx, gy)
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.path_pub.publish(msg)

def main():
    rclpy.init()
    node = DStarPatrolNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
