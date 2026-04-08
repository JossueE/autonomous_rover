#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

import tf2_geometry_msgs  # noqa: F401


def catmull_rom(p0, p1, p2, p3, t):
    t2 = t * t
    t3 = t2 * t
    x0, y0 = p0
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3

    x = 0.5 * (2*x1 + (-x0 + x2)*t + (2*x0 - 5*x1 + 4*x2 - x3)*t2 + (-x0 + 3*x1 - 3*x2 + x3)*t3)
    y = 0.5 * (2*y1 + (-y0 + y2)*t + (2*y0 - 5*y1 + 4*y2 - y3)*t2 + (-y0 + 3*y1 - 3*y2 + y3)*t3)
    return (x, y)


class PathDrawer(Node):
    def __init__(self):
        super().__init__("path_drawer")

        self.frame_id = "odom"

        # Spline parameters (tweakable)
        self.points_per_segment = 15
        self.min_points_for_spline = 4

        self.have_odom = False
        self.started = False
         # Store raw xy points (including anchor pose)
        self.raw_xy = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_click = self.create_subscription(PointStamped, "/clicked_point", self.click_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_callback, 20)

        self.pub_raw = self.create_publisher(Path, "/drawn_plan_raw", 10)
        self.pub_smooth = self.create_publisher(Path, "/drawn_plan", 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/move_base/local_costmap/costmap", 10)

        self.srv_clear = self.create_service(Empty, "/clear_drawn_plan", self.clear_callback)

        self.grid_msg = self.build_empty_grid()
        self.timer = self.create_timer(0.5, self.publish_grid)

        self.robot_x = 0.0
        self.robot_y = 0.0

        self.get_logger().info(
            "PathDrawer ready.\n"
            "- RViz: use 'Publish Point' to add points\n"
            "- /drawn_plan_raw: raw clicked points\n"
            "- /drawn_plan: smoothed spline\n"
            "- /move_base/local_costmap/costmap: empty grid published periodically\n"
            "- Clear: ros2 service call /clear_drawn_plan std_srvs/srv/Empty {}"
        )

    def odom_callback(self, msg: Odometry):
        """Cache the latest robot pose used to anchor a newly drawn path."""
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        self.have_odom = True

    def clear_callback(self, request, response):
        """Clear the stored path so the next click starts a brand-new route."""
        self.raw_xy = []
        self.started = False
        # publish empty paths so RViz clears
        self.pub_raw.publish(self.make_path_msg([]))
        self.pub_smooth.publish(self.make_path_msg([]))
        self.get_logger().warn("Cleared drawn plans.")
        return response

    def click_callback(self, msg: PointStamped):
        """Transform the clicked point to odom and append it to the full path."""
        if not self.have_odom:
            self.get_logger().warn("Ignoring click: no /odom yet.")
            return

        try:
            click_in_odom = self.tf_buffer.transform(
                msg,
                self.frame_id,
                timeout=Duration(seconds=0.2),
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Ignoring click: cannot transform {msg.header.frame_id or '<empty>'} -> {self.frame_id}: {ex}"
            )
            return

         # 1) Anchor at current robot pose (first time only)
        if not self.started:
            self.raw_xy.append((self.robot_x, self.robot_y))
            self.started = True
            self.get_logger().info(f"Anchored at robot pose ({self.robot_x:.2f}, {self.robot_y:.2f}).")

        # 2) Append clicked point
        self.raw_xy.append((float(click_in_odom.point.x), float(click_in_odom.point.y)))

        # Publish raw
        self.pub_raw.publish(self.make_path_msg(self.raw_xy))

        # Publish smooth
        smooth_xy = self.smooth_path(self.raw_xy)
        self.pub_smooth.publish(self.make_path_msg(smooth_xy))

        self.get_logger().info(
            f"Added point in {self.frame_id} ({click_in_odom.point.x:.2f}, {click_in_odom.point.y:.2f}). "
            f"Total raw: {len(self.raw_xy)}"
        )

    def build_empty_grid(self) -> OccupancyGrid:
        """Create an all-free local costmap so the controller always has a grid."""
        grid = OccupancyGrid()
        grid.header.frame_id = "odom"

        grid.info.resolution = 0.05
        grid.info.width = 200
        grid.info.height = 200

        grid.info.origin.position.x = -5.0
        grid.info.origin.position.y = -5.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.x = 0.0
        grid.info.origin.orientation.y = 0.0
        grid.info.origin.orientation.z = 0.0
        grid.info.origin.orientation.w = 1.0

        grid.data = [0] * (grid.info.width * grid.info.height)
        return grid

    def publish_grid(self):
        """Republish the empty grid with an updated timestamp."""
        self.grid_msg.header.stamp = self.get_clock().now().to_msg()
        self.grid_pub.publish(self.grid_msg)

    def make_path_msg(self, xy_list):
        """Convert a list of (x, y) tuples into a ROS Path in odom."""
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = now

        for (x, y) in xy_list:
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = now
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        return path

    def smooth_path(self, raw_xy):
        """
        Returns a smoothed list of (x,y).
        If not enough points, returns raw.
        """
                
        n = len(raw_xy)
        if n < 2:
            return raw_xy

        # With 2-3 points, just densify linearly (still nice for MPC)
        if n < self.min_points_for_spline:
            return self.linear_densify(raw_xy, per_seg=self.points_per_segment)

        pts = raw_xy
        out = []

         # For endpoints, we "extend" by repeating end points (common trick)
        for i in range(n - 1):
            # segment goes from p1 to p2, using p0,p1,p2,p3 context
            p1 = pts[i]
            p2 = pts[i + 1]
            p0 = pts[i - 1] if i - 1 >= 0 else pts[i]
            p3 = pts[i + 2] if i + 2 < n else pts[i + 1]

            # sample this segment
            for s in range(self.points_per_segment):
                t = s / float(self.points_per_segment)
                out.append(catmull_rom(p0, p1, p2, p3, t))

        # ensure the final raw point is included
        out.append(pts[-1])

        # optional: remove points that are too close (keeps /drawn_plan lighter)
        out = self.prune_close(out, min_dist=0.02)
        return out

    def linear_densify(self, xy, per_seg=10):
        """Insert evenly spaced samples between consecutive waypoints."""
        out = []
        for i in range(len(xy) - 1):
            x0, y0 = xy[i]
            x1, y1 = xy[i + 1]
            for s in range(per_seg):
                a = s / float(per_seg)
                out.append((x0*(1-a) + x1*a, y0*(1-a) + y1*a))
        out.append(xy[-1])
        return out

    def prune_close(self, xy, min_dist=0.02):
        """Remove consecutive samples that are closer than min_dist."""
        if not xy:
            return xy
        out = [xy[0]]
        for p in xy[1:]:
            x0, y0 = out[-1]
            x1, y1 = p
            if (x1-x0)*(x1-x0) + (y1-y0)*(y1-y0) >= min_dist*min_dist:
                out.append(p)
        return out


def main():
    rclpy.init()
    node = PathDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
