# automation
Automation scripts for regression

Automation Scripts for VINS & ROS2 Deployment
Overview
This repository contains automation scripts to streamline the setup, execution, and monitoring of VINS (Visual-Inertial Navigation System) and ROS2-based components inside a Dockerized environment.

The scripts include:
- vio_automation_script.py → Automates Docker setup, ROS2 execution, and tmux session management for running VINS and related components.
- vio_logs_analysis.py → Analyzes log files (vins.log, mavlink.log, vio.log) for errors, crashes, missing data, and inconsistencies.

Features
- Automated ROS2 & VINS Execution:
	- Starts and manages a Docker container for vision-computer.
	- Ensures the correct ROS2 environment is sourced before running commands.
	- Uses tmux to split terminal panes for monitoring different components.
- Robust Log Management:
	- Creates & maintains logs inside and outside the container.
	- Runs ros2 bag play and captures output for debugging.
	- Stores logs in ~/automation-logs (host) and /home/spearuav/automation-logs (container).
- Automated Error Analysis:
	- Detects crashes, missing topics, timestamp inconsistencies, and failures.
	- Summarizes critical issues in analysis_results.txt.

Installation & Setup
- Prerequisites
	Ensure the following are installed on your machine:
	- Docker (with NVIDIA support if using GPU)
	- ROS2 Humble
	- Python 3.8+
	- tmux
	- Git

- Clone the Repository:
git clone https://github.com/spearuav/automation.git
cd automation

Usage
1. Run the Automation Script:
python3 vio_automation_script.py

This script will:
- Start the Docker container (vision-computer).
- Set up ROS2 environment inside the container.
- Launch VINS, ROS2 topics, and rosbag playback inside a tmux session.
- Save logs for debugging.

To view the tmux session, run:
tmux attach-session -t test_session

2. Analyze Logs for Issues
To check logs for errors, crashes, or missing data:
python3 vio_logs_analysis.py

- This script will generate analysis_results.txt with:
	- Errors or failures detected.
	- Missing ROS topics.
	- Timestamp inconsistencies.

Log File Locations
- Inside Docker: /home/spearuav/automation-logs/
- Outside Docker: ~/automation-logs/

Troubleshooting
- Docker container is not running?
Run:
docker ps

If it's not listed, restart the container:
docker restart vision-computer

- ROS2 commands not found inside Docker?
Run:
source /mnt/sd/spearuav/ninox-viper-payload-sw/ros2_ws/install/setup.bash

- Log analysis shows missing data?
Check if ROS2 topics are publishing:
ros2 topic list

Contributing
Want to improve these scripts? Contributions are welcome!
- Fork the repository
- Create a feature branch
- Submit a pull request