#!/usr/bin/env python3

import rclpy
from daly_bms import DalyBMS


def main(args=None):
    rclpy.init(args=args)
    daly_bms = DalyBMS()
    daly_bms.get_logger().info(f"Starting Daly BMS Node{ daly_bms.get_name() }")
    daly_bms.setup()
    rclpy.spin(daly_bms)
    daly_bms.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
