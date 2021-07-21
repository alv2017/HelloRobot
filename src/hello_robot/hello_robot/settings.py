import os
from pathlib import Path

WS_DIR = '/home/ros2/ROS2/robot_ws'

# DB Location

DB_DIR_NAME = 'data'

DB_FILE_NAME = 'robot.sqlite3'

DB_FILE = os.path.join(WS_DIR, DB_DIR_NAME, DB_FILE_NAME)

# DB Tables
VELOCITYLOG = 'velocitylog'
PROCESSING = 'processing'
