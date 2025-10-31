#!/usr/bin/python3
import subprocess
import json
import os
import argparse
import asyncio
import time
import sys


"""
high level:
- we want to rsync this code to the robot running docker to /home/ubuntu/<user>/Lslidar_ROS2_driver
- then docker cp to the running image
- colcon build manually in the image

params needed:
- local user
- user@ip of robot
- docker image tag
"""


class Rsync:
    def __init__(self, robot_ip: str, container_name: str):
        self.robot_ip = robot_ip
        self.container_name = container_name
        self.curr_user = os.getlogin()
        self.target_path = f"/home/ubuntu/{self.curr_user}"

    async def rsync_inotifywait(self):
        command = (
            "inotifywait --recursive --monitor --event modify,move,create,delete "
            "--exclude 'log/|/.git'  ."
        )
        proc = await asyncio.create_subprocess_shell(
            command, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE
        )
        try:
            while True:
                # inotifywait prints only when file is changed and
                # read waits for at least one printed byte
                _ = await proc.stdout.read(n=1000000)
                print("Change detected, synchronizing")
                self.run_rsync()
                time.sleep(1)
        except asyncio.CancelledError:
            proc.terminate()

    def run_rsync(self):
        command = (
            f'rsync -avcP -e "ssh -o ConnectTimeout=10" '
            f"--chmod=Du=rwx,Dg=rwx,Do=rwx ../Lslidar_ROS2_driver ubuntu@{self.robot_ip}:{self.target_path}/"
        )
        try:
            subprocess.run(command, shell=True)  # , capture_output=True, check=True)
        except subprocess.CalledProcessError as e:
            print(f"ubuntu@{self.robot_ip}SYNCING ... ERROR")
            print("rsync failed", e.stderr)
            return
        print(f"ubuntu@{self.robot_ip} SYNCING ... OK")

        # copy to the running docker automatically
        docker_ps = subprocess.run(
            ["ssh", f"ubuntu@{self.robot_ip}", "docker ps --format json "],
            capture_output=True,
        )
        docker_ps_containers = list(
            filter(lambda x: x != "", docker_ps.stdout.decode("utf-8").split("\n"))
        )
        found = False
        for container in docker_ps_containers:
            container_json = json.loads(container)
            container_name = container_json.get("Names", "")
            image_name = container_json.get("Image", "")
            if container_name == self.container_name:
                found = True
                break

        if not found:
            print("Appropriate docker image not found. Not copying repo to image")
            return

        subprocess.run(
            [
                "ssh",
                f"ubuntu@{self.robot_ip}",
                f"docker cp {self.target_path}/Lslidar_ROS2_driver "
                f"{self.container_name}:/home/ubuntu/",
            ],
        )
        print(f"{self.container_name} DOCKER CP ... OK")


def main():
    parser = argparse.ArgumentParser(
        prog="rsync.py",
        description="Syncs the lslidar_driver folder to the robot's docker image. Image must have the name specified here",
    )

    parser.add_argument("robot_ip", help="robot ip address")
    parser.add_argument(
        "-c",
        "--container_name",
        action="store",
        help="container name to rsync this directory to",
        default=os.getlogin(),
    )
    args = parser.parse_args(sys.argv[1:])
    print(args)
    rsync = Rsync(args.robot_ip, args.container_name)
    rsync.run_rsync()
    loop = asyncio.get_event_loop()
    future = asyncio.wait_for(rsync.rsync_inotifywait(), 100000)
    try:
        loop.run_until_complete(future)
    except asyncio.exceptions.TimeoutError:
        print("Maximum script time exceeded")


if __name__ == "__main__":
    main()
