import heapq
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .geometry import yaw_to_quaternion

GridCell = Tuple[int, int]


class GridPlanner(Node):
    """A conservative A* planner over nav_msgs/OccupancyGrid."""

    def __init__(self):
        super().__init__("m20pro_grid_planner")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("pose_topic", "/m20pro_tcp_bridge/map_pose")
        self.declare_parameter("goal_topic", "/planner_goal")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("inflate_radius_m", 0.45)
        self.declare_parameter("allow_unknown", False)
        self.declare_parameter("path_downsample", 3)
        self.declare_parameter("snap_to_nearest_free", True)
        self.declare_parameter("snap_search_radius_m", 0.8)

        self.map_msg: Optional[OccupancyGrid] = None
        self.pose_msg: Optional[PoseStamped] = None
        self.inflated: List[bool] = []
        self.goal_subscriptions = []
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.path_pub = self.create_publisher(Path, str(self.get_parameter("path_topic").value), 10)
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            map_qos,
        )
        self.create_subscription(PoseStamped, str(self.get_parameter("pose_topic").value), self._on_pose, 10)
        goal_topics = self._goal_topics()
        for topic in goal_topics:
            self.goal_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    topic,
                    lambda msg, topic=topic: self._on_goal(msg, topic),
                    10,
                )
            )
        self.get_logger().info(
            "grid planner waiting for map, current pose, and goal topics: %s"
            % ", ".join(goal_topics)
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.inflated = self._inflate_obstacles(msg)
        self.get_logger().info("map received: %dx%d resolution %.3f" % (
            msg.info.width, msg.info.height, msg.info.resolution))

    def _on_pose(self, msg: PoseStamped) -> None:
        self.pose_msg = msg

    def _on_goal(self, goal: PoseStamped, topic: str) -> None:
        self.get_logger().info(
            "goal received from %s: x=%.2f y=%.2f frame=%s"
            % (
                topic,
                goal.pose.position.x,
                goal.pose.position.y,
                goal.header.frame_id or "<empty>",
            )
        )
        if self.map_msg is None or self.pose_msg is None:
            self.get_logger().warning("cannot plan yet: missing map or robot pose")
            return
        start = self._world_to_cell(self.pose_msg.pose.position.x, self.pose_msg.pose.position.y)
        target = self._world_to_cell(goal.pose.position.x, goal.pose.position.y)
        if start is None or target is None:
            self.get_logger().warning("start or goal is outside map bounds")
            return
        raw_start = start
        raw_target = target
        start = self._snap_if_needed(start, "start")
        target = self._snap_if_needed(target, "goal")
        if start is None or target is None:
            self.get_logger().warning("start or goal is blocked and no nearby free cell was found")
            return
        if start != raw_start or target != raw_target:
            self.get_logger().info("planning with snapped cells start=%s goal=%s" % (start, target))
        cells = self._astar(start, target)
        if not cells:
            self.get_logger().warning("no grid path found")
            return
        path = self._cells_to_path(cells, goal)
        self.path_pub.publish(path)
        self.get_logger().info("published path with %d poses" % len(path.poses))

    def _astar(self, start: GridCell, goal: GridCell) -> List[GridCell]:
        if self._blocked(start) or self._blocked(goal):
            return []
        open_heap: List[Tuple[float, GridCell]] = []
        heapq.heappush(open_heap, (0.0, start))
        came_from: Dict[GridCell, GridCell] = {}
        g_score: Dict[GridCell, float] = {start: 0.0}
        closed = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal:
                return self._reconstruct(came_from, current)
            closed.add(current)
            for neighbor, cost in self._neighbors(current):
                if neighbor in closed or self._blocked(neighbor):
                    continue
                tentative = g_score[current] + cost
                if tentative < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f_score = tentative + self._heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f_score, neighbor))
        return []

    def _neighbors(self, cell: GridCell) -> List[Tuple[GridCell, float]]:
        x, y = cell
        result = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nxt = (x + dx, y + dy)
                if self._in_bounds(nxt):
                    result.append((nxt, math.hypot(dx, dy)))
        return result

    def _inflate_obstacles(self, msg: OccupancyGrid) -> List[bool]:
        width = msg.info.width
        height = msg.info.height
        inflated = [False] * (width * height)
        radius_cells = int(math.ceil(float(self.get_parameter("inflate_radius_m").value) / msg.info.resolution))
        occupied = int(self.get_parameter("occupied_threshold").value)
        for y in range(height):
            for x in range(width):
                value = msg.data[y * width + x]
                if value >= occupied or (value < 0 and not bool(self.get_parameter("allow_unknown").value)):
                    for iy in range(max(0, y - radius_cells), min(height, y + radius_cells + 1)):
                        for ix in range(max(0, x - radius_cells), min(width, x + radius_cells + 1)):
                            if math.hypot(ix - x, iy - y) <= radius_cells:
                                inflated[iy * width + ix] = True
        return inflated

    def _blocked(self, cell: GridCell) -> bool:
        if not self._in_bounds(cell) or self.map_msg is None:
            return True
        x, y = cell
        return self.inflated[y * self.map_msg.info.width + x]

    def _snap_if_needed(self, cell: GridCell, label: str) -> Optional[GridCell]:
        if not self._blocked(cell):
            return cell
        if not bool(self.get_parameter("snap_to_nearest_free").value):
            return None
        snapped = self._nearest_free_cell(cell)
        if snapped is not None:
            self.get_logger().warning("%s cell %s was blocked, snapped to %s" % (label, cell, snapped))
        return snapped

    def _nearest_free_cell(self, cell: GridCell) -> Optional[GridCell]:
        assert self.map_msg is not None
        radius_cells = int(
            math.ceil(float(self.get_parameter("snap_search_radius_m").value) / self.map_msg.info.resolution)
        )
        best: Optional[GridCell] = None
        best_dist = float("inf")
        cx, cy = cell
        for y in range(max(0, cy - radius_cells), min(self.map_msg.info.height, cy + radius_cells + 1)):
            for x in range(max(0, cx - radius_cells), min(self.map_msg.info.width, cx + radius_cells + 1)):
                candidate = (x, y)
                if self._blocked(candidate):
                    continue
                dist = math.hypot(x - cx, y - cy)
                if dist < best_dist:
                    best = candidate
                    best_dist = dist
        return best

    def _in_bounds(self, cell: GridCell) -> bool:
        if self.map_msg is None:
            return False
        x, y = cell
        return 0 <= x < self.map_msg.info.width and 0 <= y < self.map_msg.info.height

    def _world_to_cell(self, x: float, y: float) -> Optional[GridCell]:
        if self.map_msg is None:
            return None
        origin = self.map_msg.info.origin.position
        res = self.map_msg.info.resolution
        gx = int((x - origin.x) / res)
        gy = int((y - origin.y) / res)
        cell = (gx, gy)
        return cell if self._in_bounds(cell) else None

    def _cell_to_world(self, cell: GridCell) -> Tuple[float, float]:
        assert self.map_msg is not None
        origin = self.map_msg.info.origin.position
        res = self.map_msg.info.resolution
        return origin.x + (cell[0] + 0.5) * res, origin.y + (cell[1] + 0.5) * res

    def _cells_to_path(self, cells: List[GridCell], goal: PoseStamped) -> Path:
        assert self.map_msg is not None
        step = max(1, int(self.get_parameter("path_downsample").value))
        sampled = cells[::step]
        if sampled[-1] != cells[-1]:
            sampled.append(cells[-1])
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_msg.header.frame_id or "map"
        for idx, cell in enumerate(sampled):
            x, y = self._cell_to_world(cell)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if idx + 1 < len(sampled):
                nx, ny = self._cell_to_world(sampled[idx + 1])
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 2.0 * math.atan2(goal.pose.orientation.z, goal.pose.orientation.w)
            pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)
        return path

    @staticmethod
    def _heuristic(a: GridCell, b: GridCell) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _reconstruct(came_from: Dict[GridCell, GridCell], current: GridCell) -> List[GridCell]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _goal_topics(self) -> List[str]:
        configured = str(self.get_parameter("goal_topic").value)
        topics: List[str] = []
        for topic in (configured, "/goal_pose", "/move_base_simple/goal"):
            if topic and topic not in topics:
                topics.append(topic)
        return topics


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = GridPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
