import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np
import math
import argparse
import sys

# Custom service (must be built beforehand)
from robot_interfaces.srv import GetSensorPose2D

class SensorField(Node):
    def __init__(self, 
                 antenna_type="omnidirectional", 
                 freq=2.4e9, 
                 tx_power_dbm=0.0,
                 yaw_deg=0.0, 
                 pitch_deg=0.0):
        super().__init__('rf_antenna_publisher')

        # ---------------------------
        # (A) Parameter settings
        # ---------------------------
        self.antenna_type = antenna_type
        self.freq_hz = freq
        self.tx_power_dbm = tx_power_dbm
        self.reflection_coef = 0.7  # Ground reflection coefficient (linear)
        self.num_points = 100000      # Number of randomly generated points
        self.max_radius = 50.0      # Maximum propagation distance

        # Antenna position
        self.real_antenna_pos = np.array([0.0, 0.0, 1.0])
        self.image_antenna_pos = np.array([0.0, 0.0, -1.0])  # Image antenna (reflection)

        # Antenna orientation (Yaw/Pitch) → Main lobe axis vector self.antenna_dir
        self.antenna_dir = self.compute_antenna_direction(yaw_deg, pitch_deg)

        self.get_logger().info(f"[RFAntennaPublisher] type={antenna_type}, freq={freq}, tx={tx_power_dbm} dBm")
        self.get_logger().info(f"Yaw={yaw_deg} deg, Pitch={pitch_deg} deg, dir={self.antenna_dir}")

        # ---------------------------
        # (B) Point cloud publisher
        # ---------------------------
        self.publisher_ = self.create_publisher(PointCloud2, 'rf_field', 10)
        self.timer = self.create_timer(0.5, self.publish_rf_field)

        # ---------------------------
        # (C) Service server
        # ---------------------------
        self.srv_server = self.create_service(GetSensorPose2D, 'get_sensor', self.get_rx_power_callback)

    # ---------------------------
    # (A1) Calculate antenna direction vector
    # ---------------------------
    def compute_antenna_direction(self, yaw_deg, pitch_deg):
        """
        Returns unit vector for antenna main lobe axis by rotating from initial direction
        z-axis positive (0,0,1) with Yaw (around Z-axis) and Pitch (around Y-axis).
         - yaw_deg: Around Z-axis (heading)
         - pitch_deg: Around Y-axis
        Roll (around X-axis) can be added if desired.
        """
        yaw_rad = math.radians(yaw_deg)
        pitch_rad = math.radians(pitch_deg)

        # Initial vector: z-axis
        v = np.array([0.0, 0.0, 1.0], dtype=float)

        # Rotation matrix Rz(yaw)
        Rz = np.array([
            [ math.cos(yaw_rad), -math.sin(yaw_rad), 0],
            [ math.sin(yaw_rad),  math.cos(yaw_rad), 0],
            [               0,                 0,    1],
        ], dtype=float)

        # Rotation matrix Ry(pitch)
        Ry = np.array([
            [ math.cos(pitch_rad), 0, math.sin(pitch_rad)],
            [ 0,                   1,                  0],
            [-math.sin(pitch_rad), 0, math.cos(pitch_rad)],
        ], dtype=float)

        # Combined rotation R = Rz * Ry (order adjusted according to application)
        R = Rz @ Ry

        # Rotated vector
        rotated = R @ v
        norm_val = np.linalg.norm(rotated)
        if norm_val < 1e-12:
            return np.array([0,0,1], dtype=float)
        return rotated / norm_val

    # =======================================================
    # (B) Publish point cloud
    # =======================================================
    def publish_rf_field(self):
        # 1) Real antenna (direct wave)
        x1, y1, z1, p_db1 = self.generate_points(antenna_pos=self.real_antenna_pos, reflection_lin=1.0)

        # 2) Image antenna (reflected wave)
        x2, y2, z2, p_db2 = self.generate_points(antenna_pos=self.image_antenna_pos, reflection_lin=self.reflection_coef)

        # Combine
        x_all = np.concatenate((x1, x2))
        y_all = np.concatenate((y1, y2))
        z_all = np.concatenate((z1, z2))
        p_db_all = np.concatenate((p_db1, p_db2))

        # Remove z<0 (no radio waves underground)
        mask = (z_all >= 0)
        x_all = x_all[mask]
        y_all = y_all[mask]
        z_all = z_all[mask]
        p_db_all = p_db_all[mask]

        # Normalize dB(-100 to 0) → 0.0 to 1.0
        intensity = (p_db_all + 100.0) / 100.0
        intensity = np.clip(intensity, 0.0, 1.0)

        # Create PointCloud2
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.height = 1
        msg.width = len(x_all)
        msg.is_dense = False
        msg.is_bigendian = False
        msg.point_step = 16  # (x, y, z, intensity) four float32 values
        msg.row_step = msg.point_step * len(x_all)

        msg.fields = [
            PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        data_list = []
        for i in range(len(x_all)):
            data_list.append(struct.pack('ffff', x_all[i], y_all[i], z_all[i], intensity[i]))
        msg.data = b"".join(data_list)

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published {len(x_all)} points")

    def generate_points(self, antenna_pos, reflection_lin):
        """
        Generate random points (r,theta,phi) and calculate received power [dB] at those points
        (considering antenna orientation self.antenna_dir and directivity)
        """
        # (1) Generate random (r, theta, phi)
        r     = np.random.uniform(0.5, self.max_radius, self.num_points)
        theta = np.random.uniform(0.0, np.pi, self.num_points)
        phi   = np.random.uniform(0.0, 2.0*np.pi, self.num_points)

        # (2) Convert to Cartesian coordinates (offset by antenna position)
        x = r*np.sin(theta)*np.cos(phi) + antenna_pos[0]
        y = r*np.sin(theta)*np.sin(phi) + antenna_pos[1]
        z = r*np.cos(theta)             + antenna_pos[2]

        # (3) Relative vector rel
        rel_x = x - antenna_pos[0]
        rel_y = y - antenna_pos[1]
        rel_z = z - antenna_pos[2]
        rr = np.sqrt(rel_x**2 + rel_y**2 + rel_z**2)
        rr = np.clip(rr, 1e-12, None)

        # (4) Calculate angle with antenna direction self.antenna_dir
        #     dot = rel·dir
        dot_vals = rel_x*self.antenna_dir[0] + rel_y*self.antenna_dir[1] + rel_z*self.antenna_dir[2]
        cos_vals = dot_vals / (rr * np.linalg.norm(self.antenna_dir))  # dir is unit vector so norm=1, but just in case...
        cos_vals = np.clip(cos_vals, -1.0, 1.0)
        theta_ant = np.arccos(cos_vals)  # 0 <= theta_ant <= π

        # (5) Directivity gain [dBi]
        gain_db = self.compute_gain_db(theta_ant)

        # (6) Reflection coefficient (linear→dB)
        reflection_db = 10.0 * math.log10(reflection_lin)

        # (7) Free-space path loss FSPL(dB)
        fspl_db = 20.0*np.log10(rr) + 20.0*np.log10(self.freq_hz) - 147.55

        # (8) Received power p_rx_db
        p_rx_db = self.tx_power_dbm + gain_db + reflection_db - fspl_db
        return x, y, z, p_rx_db

    def compute_gain_db(self, theta):
        """
        Returns directivity gain [dBi] based on antenna type
         - omnidirectional: 0 dBi
         - directional: cos²(θ) (linear → dB)
         - yagi: Gaussian beam
        """
        if self.antenna_type == "omnidirectional":
            return np.zeros_like(theta)

        elif self.antenna_type == "directional":
            g_lin = np.cos(theta)**2
            g_db = 10.0*np.log10(np.clip(g_lin, 1e-12, None))
            return g_db

        elif self.antenna_type == "yagi":
            theta_0 = np.pi/6.0  # 30 degrees
            g_lin = np.exp(- (theta/theta_0)**2)
            g_db = 10.0*np.log10(np.clip(g_lin, 1e-12, None))
            return g_db

        else:
            self.get_logger().warn("Unknown antenna type -> use omnidirectional(0dBi)")
            return np.zeros_like(theta)

    # =======================================================
    # (C) Service (x,y,z) -> received power [dB]
    # =======================================================
    def get_rx_power_callback(self, request, response):
        """
        Calculate and return the combined received power [dB] of "direct wave + reflected wave" at arbitrary coordinates (x,y,z).
        If z<0, return -999dB.
        """
        xq, yq, zq = request.x, request.y, 0.5

        # No radio waves underground
        if zq < 0:
            response.rx_db = -999.0
            return response

        # (1) Direct wave (real antenna)
        direct_db = self.compute_rx_power_db_for_point(xq, yq, zq, self.real_antenna_pos, 1.0)
        # (2) Reflected wave (image antenna)
        refl_db   = self.compute_rx_power_db_for_point(xq, yq, zq, self.image_antenna_pos, self.reflection_coef)

        # (3) Power combination (linear addition -> dB)
        p_direct_lin = 10.0**(direct_db/10.0)
        p_refl_lin   = 10.0**(refl_db/10.0)
        p_total_lin  = p_direct_lin + p_refl_lin
        p_total_db   = 10.0 * math.log10(p_total_lin)

        response.data = p_total_db
        return response

    def compute_rx_power_db_for_point(self, x, y, z, antenna_pos, reflection_lin):
        """
        Calculate received power [dB] at one point (x,y,z) (antenna directivity + FSPL + reflection)
        """
        rel_x = x - antenna_pos[0]
        rel_y = y - antenna_pos[1]
        rel_z = z - antenna_pos[2]
        rr = math.sqrt(rel_x**2 + rel_y**2 + rel_z**2)
        if rr < 1e-12:
            rr = 1e-12

        # Angle with antenna direction self.antenna_dir
        dot_val = rel_x*self.antenna_dir[0] + rel_y*self.antenna_dir[1] + rel_z*self.antenna_dir[2]
        cos_val = dot_val / (rr * np.linalg.norm(self.antenna_dir))
        cos_val = max(-1.0, min(1.0, cos_val))
        theta = math.acos(cos_val)

        # Directivity gain [dBi]
        gain_db = 0.0
        if self.antenna_type == "omnidirectional":
            gain_db = 0.0
        elif self.antenna_type == "directional":
            g_lin = (math.cos(theta))**2
            gain_db = 10.0*math.log10(max(g_lin, 1e-12))
        elif self.antenna_type == "yagi":
            theta_0 = math.pi/6.0
            g_lin = math.exp(- (theta/theta_0)**2)
            gain_db = 10.0*math.log10(max(g_lin, 1e-12))

        # Reflection coefficient (linear→dB)
        reflection_db = 10.0 * math.log10(reflection_lin)

        # FSPL(dB)
        fspl_db = 20.0*math.log10(rr) + 20.0*math.log10(self.freq_hz) - 147.55

        # Received power [dB]
        p_rx_db = self.tx_power_dbm + gain_db + reflection_db - fspl_db
        return p_rx_db


def main():
    parser = argparse.ArgumentParser(description="RF Sim with orientation + reflection + service")
    parser.add_argument('--antenna_type', type=str, default="omnidirectional",
                        choices=["omnidirectional", "directional", "yagi"],
                        help="Antenna model type")
    parser.add_argument('--freq', type=float, default=2.4e9, help="Frequency [Hz]")
    parser.add_argument('--tx_power_dbm', type=float, default=0.0, help="Tx power [dBm]")
    parser.add_argument('--yaw_deg', type=float, default=0.0, help="Yaw around Z-axis [deg]")
    parser.add_argument('--pitch_deg', type=float, default=0.0, help="Pitch around Y-axis [deg]")
    args, _ = parser.parse_known_args()

    rclpy.init(args=sys.argv)
    node = SensorField(
        antenna_type=args.antenna_type,
        freq=args.freq,
        tx_power_dbm=args.tx_power_dbm,
        yaw_deg=args.yaw_deg,
        pitch_deg=args.pitch_deg
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
