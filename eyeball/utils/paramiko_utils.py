import logging
import socket
import time
from dataclasses import dataclass

import paramiko

SSH_PORT = 22


@dataclass
class ServerConfig:
    hostname: str
    username: str
    password: str
    python_environment: str
    config_path: str
    self_hostname: str
    wait_seconds: int | None = 3
    check_time_sync: bool = True  # check if chronyd is running and synced, to make sure the time is synced.


def check_port_availability(hostname: str, port: int, timeout: int = 5) -> bool:
    """Check if the port is available on the remote server"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((hostname, port))
        sock.close()
        return result != 0  # 0 means connection successful (port occupied), non-zero means port available
    except Exception:
        return False


def kill_process_on_port(ssh: paramiko.SSHClient, port: int) -> bool:
    """Kill the process occupying the specified port"""
    try:
        # Find the process occupying the port
        stdin, stdout, stderr = ssh.exec_command(f"lsof -ti:{port}")
        pids = stdout.read().decode().strip().split("\n")

        if pids and pids[0]:  # If there's a process occupying the port
            for pid in pids:
                if pid.strip():
                    logging.info(f"Killing process {pid} on port {port}")
                    ssh.exec_command(f"kill -9 {pid}")

            # Wait for the process to completely terminate
            time.sleep(0.5)
            return True
        return False
    except Exception as e:
        logging.error(f"Failed to kill process on port {port}: {e}")
        return False


def check_chronyd_status(ssh: paramiko.SSHClient, server_config: ServerConfig):
    # Check if chronyd is running and synced
    logging.info("Checking chronyd status on remote host...")
    stdin, stdout, stderr = ssh.exec_command("chronyc sources -v")
    chrony_output = stdout.read().decode()

    logging.info(f"Remote host chronyd status: {chrony_output}")
    if server_config.self_hostname not in chrony_output:
        raise RuntimeError(
            f"Remote host:{server_config.hostname} chronyd is not synced with the main compiuter:{server_config.self_hostname}."
        )


class RemoteProcessHandle:
    """
    A handle to manage the lifecycle of the remote process.
    Ensures the SSH connection is closed and the remote process is terminated.
    """

    def __init__(self, ssh_client: paramiko.SSHClient, pid: int, hostname: str):
        self._ssh = ssh_client
        self._pid = pid
        self._hostname = hostname

    def kill(self):
        """Terminates the remote process and closes the SSH connection."""
        if not self._ssh:
            return  # Already terminated

        # A single try/finally block is the cleanest way to ensure resource cleanup.
        try:
            # Attempt to kill the process if the connection is still active.
            transport = self._ssh.get_transport()
            if transport and transport.is_active():
                logging.info(f"Terminating remote process {self._pid} on {self._hostname}")
                self._ssh.exec_command(f"kill -9 {self._pid}")
        finally:
            # Regardless of the outcome above, always close the connection
            # and mark the handle as terminated.
            self._ssh.close()
            self._ssh = None
            logging.info(f"SSH connection to {self._hostname} closed.")

    def __repr__(self):
        status = "Terminated"
        if self._ssh:
            transport = self._ssh.get_transport()
            if transport and transport.is_active():
                status = "Active"
        return f"<RemoteProcessHandle for PID {self._pid} on {self._hostname} ({status})>"