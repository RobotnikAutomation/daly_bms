#!/usr/bin/env python3
# Copyright 2024 Robotnik Automation S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# @maintanier Guillem Gari  <ggari@robotnik.es> Robotnik Automation S.L.

"""
Main entry point for the Daly BMS ROS2 node.

This module initializes and runs the Daly BMS (Battery Management System)
ROS2 node. It sets up the ROS2 environment, creates an instance of the
DalyBMS node, and manages the node's lifecycle.

The main function in this module performs the following steps:
1. Initializes the ROS2 client library.
2. Creates an instance of the DalyBMS node.
3. Logs the start of the node.
4. Calls the setup method of the DalyBMS node.
5. Spins the node to keep it alive and responsive to callbacks.
6. Handles the shutdown process when the node is terminated.

This script is designed to be the entry point for running the Daly BMS
node, either standalone or as part of a larger ROS2 system.

Functions
---------
    main: The main entry point for running the Daly BMS node.

Dependencies
------------
    - rclpy: The ROS2 Python client library.
    - daly_bms: The module containing the DalyBMS node implementation.

Notes
-----
    Ensure that the ROS2 environment is properly set up before running
    this script.

"""

import rclpy
from .daly_bms import DalyBMS


def main(args=None):
    """
    Initialize and run the Daly BMS ROS2 node.

    This function performs the following steps:
    1. Initializes the ROS2 client library.
    2. Creates an instance of the DalyBMS node.
    3. Logs the start of the node.
    4. Sets up the DalyBMS node.
    5. Spins the node to process callbacks.
    6. Handles node shutdown when interrupted.

    Parameters
    ----------
    args : list, optional
        Command line arguments passed to the ROS2 program.
        Default is None, which uses sys.argv.

    Notes
    -----
    This function is designed to be the main entry point for the Daly BMS
    node. It handles the complete lifecycle of the node from initialization
    to shutdown.

    """
    rclpy.init(args=args)
    daly_bms = DalyBMS()
    daly_bms.get_logger().info(
        f"Starting Daly BMS Node{ daly_bms.get_name() }"
    )
    daly_bms.setup()
    rclpy.spin(daly_bms)
    daly_bms.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
