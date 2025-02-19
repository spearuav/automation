import subprocess
import time
import os

DEBUG = 0

LOG_DIR = os.path.expanduser("~/automation-logs")  # Store logs outside the container

latest_image_id_path = os.path.join(LOG_DIR, "latest_image_id.txt")


def run_command(command):
    """Runs a shell command and waits for it to complete."""
    subprocess.run(command, shell=True, check=True)


def is_docker_running():
    """Checks if the vision-computer Docker container is running."""
    try:
        output = subprocess.check_output("docker ps | grep vision-computer", shell=True).decode()
        return "vision-computer" in output
    except subprocess.CalledProcessError:
        return False


def wait_for_docker():
    """Waits until Docker container starts before proceeding."""
    print("Waiting for Docker container to start...")
    for i in range(30):  # Max wait time: 30 seconds
        if is_docker_running():
            print("? Docker container is running! Proceeding with automation.")
            return True
        time.sleep(2)  # Check every 2 seconds
    print("? Timeout: Docker container did not start.")
    return False


def setup_logs():
    """Creates a directory for logs on the host and inside the container."""
    try:
        os.makedirs(LOG_DIR, exist_ok=True)  # Ensure directory exists on host
        for log_file in ["vins.log", "rosbag.log", "mavlink.log", "vio.log"]:
            open(os.path.join(LOG_DIR, log_file), "w").close()  # Clear old logs
    except PermissionError as e:
        print(f"? ERROR: Cannot create log directory {LOG_DIR}: {e}")
        exit(1)  # Exit script if log directory cannot be created


def start_tmux_session():
    IS_GITHUB_ACTIONS = os.getenv("GITHUB_ACTIONS") == "true"

    print("? Starting automation script...")
    setup_logs()

    # Start a new tmux session
    run_command("tmux new-session -d -s test_session")

    # Split into four panes
    run_command("tmux split-window -h")  # Split horizontally (Pane 1, 2)
    run_command("tmux split-window -v")  # Split Pane 2 into 2 (Pane 2, 3)
    run_command("tmux select-pane -t 0; tmux split-window -v")  # Split Pane 1 into 2 (Pane 1, 4)

    # Terminal 1: Manage Docker and Run VINS
    run_command("tmux send-keys -t 0 'docker stop vision-computer && docker rm vision-computer' C-m")
    if (DEBUG):
        run_command("tmux send-keys -t 0 'docker pull ghcr.io/spearuav/vision-computer:3.16.15' C-m")
    else:
        run_command("tmux send-keys -t 0 'docker pull ghcr.io/spearuav/vision-computer:latest' C-m")
    run_command(
        "tmux send-keys -t 0 'docker images | grep spearuav/vision-computer | head -n 1 | awk \"{print \\$3}\" > ~/automation-logs/latest_image_id.txt' C-m")

    time.sleep(5)
    if IS_GITHUB_ACTIONS:
        if (DEBUG):
            run_command(
                f"tmux send-keys -t 0 'docker run --privileged --runtime nvidia --network host --device /dev/video0 --device /dev/video1 --device /dev/video2 --device /dev/video3 --device /dev/ttyTHS0 --device /dev/ttyACM0 --device /dev/ttyACM1 -v /mnt/sd/spearuav/ninox-viper-payload-sw/configuration:/mnt/sd/spearuav/ninox-viper-payload-sw/configuration -v /mnt/sd/spearuav/logs:/mnt/sd/spearuav/logs -v /tmp/argus_socket:/tmp/argus_socket -v /lib/modules/5.10.104-g018a5562a:/lib/modules/5.10.104-g018a5562a  -v $HOME/.Xauthority:/root/.Xauthority:rw -v /tmp/.X11-unix:/tmp/.X11-unix -v /rosbag:/rosbag -v {LOG_DIR}:/home/spearuav/automation-logs --name vision-computer 1e0cc57adcb3' C-m")
        else:
            run_command(
                f"tmux send-keys -t 0 'image_id=$(cat ~/automation-logs/latest_image_id.txt) && docker run --privileged --runtime nvidia --network host --device /dev/video0 --device /dev/video1 --device /dev/video2 --device /dev/video3 --device /dev/ttyTHS0 --device /dev/ttyACM0 --device /dev/ttyACM1 -v /mnt/sd/spearuav/ninox-viper-payload-sw/configuration:/mnt/sd/spearuav/ninox-viper-payload-sw/configuration -v /mnt/sd/spearuav/logs:/mnt/sd/spearuav/logs -v /tmp/argus_socket:/tmp/argus_socket -v /lib/modules/5.10.104-g018a5562a:/lib/modules/5.10.104-g018a5562a -v $HOME/.Xauthority:/root/.Xauthority:rw -v /tmp/.X11-unix:/tmp/.X11-unix -v /rosbag:/rosbag -v {LOG_DIR}:/home/spearuav/automation-logs --name vision-computer $image_id' C-m")
    else:
        if (DEBUG):
            run_command(
                f"tmux send-keys -t 0 'docker run -it --privileged --runtime nvidia --network host --device /dev/video0 --device /dev/video1 --device /dev/video2 --device /dev/video3 --device /dev/ttyTHS0 --device /dev/ttyACM0 --device /dev/ttyACM1 -v /mnt/sd/spearuav/ninox-viper-payload-sw/configuration:/mnt/sd/spearuav/ninox-viper-payload-sw/configuration -v /mnt/sd/spearuav/logs:/mnt/sd/spearuav/logs -v /tmp/argus_socket:/tmp/argus_socket -v /lib/modules/5.10.104-g018a5562a:/lib/modules/5.10.104-g018a5562a  -v $HOME/.Xauthority:/root/.Xauthority:rw -v /tmp/.X11-unix:/tmp/.X11-unix -v /rosbag:/rosbag -v {LOG_DIR}:/home/spearuav/automation-logs --name vision-computer 1e0cc57adcb3' C-m")
        else:
            run_command(
                f"tmux send-keys -t 0 'image_id=$(cat ~/automation-logs/latest_image_id.txt) && docker run -it --privileged --runtime nvidia --network host --device /dev/video0 --device /dev/video1 --device /dev/video2 --device /dev/video3 --device /dev/ttyTHS0 --device /dev/ttyACM0 --device /dev/ttyACM1 -v /mnt/sd/spearuav/ninox-viper-payload-sw/configuration:/mnt/sd/spearuav/ninox-viper-payload-sw/configuration -v /mnt/sd/spearuav/logs:/mnt/sd/spearuav/logs -v /tmp/argus_socket:/tmp/argus_socket -v /lib/modules/5.10.104-g018a5562a:/lib/modules/5.10.104-g018a5562a -v $HOME/.Xauthority:/root/.Xauthority:rw -v /tmp/.X11-unix:/tmp/.X11-unix -v /rosbag:/rosbag -v {LOG_DIR}:/home/spearuav/automation-logs --name vision-computer $image_id' C-m")

    # Wait for Docker to start
    if not wait_for_docker():
        print("? Exiting due to Docker startup failure.")
        return

    # Ensure log directory exists inside the container
    if IS_GITHUB_ACTIONS:
        run_command(
            "docker exec vision-computer mkdir -p /home/spearuav/automation-logs")
    else:
        run_command("docker exec -it vision-computer mkdir -p /home/spearuav/automation-logs")

    if (DEBUG):
        run_command("tmux send-keys -t 0 'docker pull ghcr.io/spearuav/vision-computer:3.16.15' C-m")
    else:
        run_command("tmux send-keys -t 0 'docker pull ghcr.io/spearuav/vision-computer:latest' C-m")
    run_command(
        "tmux send-keys -t 0 'docker images | grep spearuav/vision-computer | head -n 1 | awk \"{print \\$3}\" > ~/automation-logs/latest_image_id.txt' C-m")

    time.sleep(5)
    if IS_GITHUB_ACTIONS:
        if (DEBUG):
            run_command(
                f"tmux send-keys -t 0 'docker run --privileged --runtime nvidia --network host --device /dev/video0 --device /dev/video1 --device /dev/video2 --device /dev/video3 --device /dev/ttyTHS0 --device /dev/ttyACM0 --device /dev/ttyACM1 -v /mnt/sd/spearuav/ninox-viper-payload-sw/configuration:/mnt/sd/spearuav/ninox-viper-payload-sw/configuration -v /mnt/sd/spearuav/logs:/mnt/sd/spearuav/logs -v /tmp/argus_socket:/tmp/argus_socket -v /lib/modules/5.10.104-g018a5562a:/lib/modules/5.10.104-g018a5562a  -v $HOME/.Xauthority:/root/.Xauthority:rw -v /tmp/.X11-unix:/tmp/.X11-unix -v /rosbag:/rosbag -v {LOG_DIR}:/home/spearuav/automation-logs --name vision-computer 1e0cc57adcb3' C-m")
        else:
            run_command(
                f"tmux send-keys -t 0 'image_id=$(cat ~/automation-logs/latest_image_id.txt) && docker run --privileged --runtime nvidia --network host --device /dev/video0 --device /dev/video1 --device /dev/video2 --device /dev/video3 --device /dev/ttyTHS0 --device /dev/ttyACM0 --device /dev/ttyACM1 -v /mnt/sd/spearuav/ninox-viper-payload-sw/configuration:/mnt/sd/spearuav/ninox-viper-payload-sw/configuration -v /mnt/sd/spearuav/logs:/mnt/sd/spearuav/logs -v /tmp/argus_socket:/tmp/argus_socket -v /lib/modules/5.10.104-g018a5562a:/lib/modules/5.10.104-g018a5562a -v $HOME/.Xauthority:/root/.Xauthority:rw -v /tmp/.X11-unix:/tmp/.X11-unix -v /rosbag:/rosbag -v {LOG_DIR}:/home/spearuav/automation-logs --name vision-computer $image_id' C-m")
    else:
        if (DEBUG):
            run_command(
                f"tmux send-keys -t 0 'docker run -it --privileged --runtime nvidia --network host --device /dev/video0 --device /dev/video1 --device /dev/video2 --device /dev/video3 --device /dev/ttyTHS0 --device /dev/ttyACM0 --device /dev/ttyACM1 -v /mnt/sd/spearuav/ninox-viper-payload-sw/configuration:/mnt/sd/spearuav/ninox-viper-payload-sw/configuration -v /mnt/sd/spearuav/logs:/mnt/sd/spearuav/logs -v /tmp/argus_socket:/tmp/argus_socket -v /lib/modules/5.10.104-g018a5562a:/lib/modules/5.10.104-g018a5562a  -v $HOME/.Xauthority:/root/.Xauthority:rw -v /tmp/.X11-unix:/tmp/.X11-unix -v /rosbag:/rosbag -v {LOG_DIR}:/home/spearuav/automation-logs --name vision-computer 1e0cc57adcb3' C-m")
        else:
            run_command(
                f"tmux send-keys -t 0 'image_id=$(cat ~/automation-logs/latest_image_id.txt) && docker run -it --privileged --runtime nvidia --network host --device /dev/video0 --device /dev/video1 --device /dev/video2 --device /dev/video3 --device /dev/ttyTHS0 --device /dev/ttyACM0 --device /dev/ttyACM1 -v /mnt/sd/spearuav/ninox-viper-payload-sw/configuration:/mnt/sd/spearuav/ninox-viper-payload-sw/configuration -v /mnt/sd/spearuav/logs:/mnt/sd/spearuav/logs -v /tmp/argus_socket:/tmp/argus_socket -v /lib/modules/5.10.104-g018a5562a:/lib/modules/5.10.104-g018a5562a -v $HOME/.Xauthority:/root/.Xauthority:rw -v /tmp/.X11-unix:/tmp/.X11-unix -v /rosbag:/rosbag -v {LOG_DIR}:/home/spearuav/automation-logs --name vision-computer $image_id' C-m")

    # Wait for Docker to start
    if not wait_for_docker():
        print("? Exiting due to Docker startup failure.")
        return

    # Ensure Docker container is running before executing commands inside it
    if not is_docker_running():
        print("? Error: Docker container is not running. Restarting container...")
        run_command("docker restart vision-computer")
        time.sleep(5)  # Give time for the container to start
        if not is_docker_running():
            print("? Error: Docker container failed to start.")
            return

    # Terminal 1: Run VINS and Log
    run_command(f"tmux send-keys -t 0 'ros2 run vins vins_node | tee {LOG_DIR}/vins.log' C-m")

    # Terminal 2: Verify and Run rosbag
    if IS_GITHUB_ACTIONS:
        run_command("tmux send-keys -t 1 'docker exec vision-computer /bin/bash' C-m")
    else:
        run_command("tmux send-keys -t 1 'docker exec -it vision-computer /bin/bash' C-m")
    run_command("tmux send-keys -t 1 'source ros2_ws/install/setup.bash' C-m")
    run_command(f"tmux send-keys -t 1 'cd /rosbag/ && ros2 bag play tt3 | tee {LOG_DIR}/rosbag.log' C-m")

    # Terminal 3: Run Mavlink logs and Log
    if IS_GITHUB_ACTIONS:
        run_command("tmux send-keys -t 2 'docker exec vision-computer /bin/bash' C-m")
    else:
        run_command("tmux send-keys -t 2 'docker exec -it vision-computer /bin/bash' C-m")
    run_command("tmux send-keys -t 2 'source ros2_ws/install/setup.bash' C-m")
    run_command(
        f"tmux send-keys -t 2 'ros2 service call /vins_enable_service viper_interfaces/srv/VioEnable \"{{\\\"enable\\\":True}}\" | tee {LOG_DIR}/mavlink.log' C-m")
    run_command(f"tmux send-keys -t 2 'ros2 topic echo /mavlink_cmds | tee -a {LOG_DIR}/mavlink.log' C-m")

    # Terminal 4: Monitor logs and Log
    run_command(f"tmux send-keys -t 3 'docker logs -f --tail 20 vision-computer | tee {LOG_DIR}/vio.log' C-m")

    print(f"? Automation started! Logs saved in {LOG_DIR}")
    print("? Use 'tmux attach-session -t test_session' to view it.")

if __name__ == "__main__":
    start_tmux_session()