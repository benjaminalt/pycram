import logging
import threading
from abc import ABC, abstractmethod

import roslibpy

from ros.rosbridge import ROSBridge


class ROSTopicPublisher(ABC):
    def __init__(self):
        ros_host, ros_port  = ROSBridge.get_ros_master_host_and_port()
        logging.info(f"ROSTopicPublisher: Connecting to ROS at {ros_host}:{ros_port}")
        self.ros_client = roslibpy.Ros(ros_host, ros_port)
        self.ros_client.run()

        self.thread = None
        self.kill_event = threading.Event()

    def __del__(self):
        self.ros_client.terminate()

    def start_publishing(self):
        logging.info(f"{self.__class__.__name__}::start_publishing: Starting publisher thread...")
        if not self.ros_client.is_connected:
            raise RuntimeError(f"{self.__class__.__name__}: Cannot start publishing, ROS client not connected")
        if self.kill_event.is_set():
            self.kill_event.clear()
        self.thread = threading.Thread(target=self._publish)
        self.thread.start()
        logging.info(f"{self.__class__.__name__}::start_publishing: Publisher thread started")

    def stop_publishing(self):
        logging.info(f"{self.__class__.__name__}::stop_publishing: Stopping publisher thread...")
        if self.thread:
            self.kill_event.set()
            self.thread.join()
            self.thread = None
            logging.info(f"{self.__class__.__name__}::stop_publishing: Publisher thread stopped")
        else:
            logging.info(f"{self.__class__.__name__}::stop_publishing: Publisher thread not running")

    def __enter__(self):
        self.start_publishing()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_publishing()

    @abstractmethod
    def _publish(self):
        pass