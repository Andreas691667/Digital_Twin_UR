import logging
import os
import socket
import sys
import threading
import time
from queue import Queue

import numpy as np

from third_party.rtde import rtde
from urinterface.recording import _recording_thread, STOP_REQUEST

_log = logging.getLogger("RobotConnection")


class RobotConnection:
    # ip_adr: <string>
    # controller_socket, dashboard_socket: <socket>
    # is_logging_data: <bool>

    def __init__(self, ip_adr,
                 controller_socket=socket.socket(socket.AF_INET, socket.SOCK_STREAM),
                 dashboard_socket=socket.socket(socket.AF_INET, socket.SOCK_STREAM)):
        self.ip_adr = ip_adr
        self.recording_thread = None
        self.recording_thread_queue = Queue()
        self.logging_process = None
        controller_port = 30003
        dashboard_port = 29999

        self.buff_size = 1024

        # Create and connect to controller and dashboard TCP/IP sockets
        self.controller_socket = controller_socket
        self.dashboard_socket = dashboard_socket
        self.dashboard_socket.connect((ip_adr, dashboard_port))
        self.controller_socket.connect((ip_adr, controller_port))

        reply = self._receive_ascii_bytes(self.dashboard_socket)
        assert reply.startswith("Connected")

    def _receive_ascii_bytes(self, socket):
        reply = socket.recv(self.buff_size)
        _log.info(f"receiving bytes:{reply}")
        reply_ascii = reply.decode(encoding="ascii")
        _log.info(f"receiving:{reply_ascii}")
        return reply_ascii

    def _send_ascii_bytes(self, msg, socket):
        to_send = bytes(msg, encoding="ASCII")
        _log.info(f"sending:{to_send}")
        socket.sendall(to_send)
        _log.info("sent.")

    def __del__(self):
        # Should it stop recording here???
        # TODO : When deleting the object, make sure that only connected sockets will be closed.
        self.dashboard_socket.shutdown(socket.SHUT_RDWR)
        self.controller_socket.shutdown(socket.SHUT_RDWR)
        self.dashboard_socket.close()
        self.controller_socket.close()

    def _ensure_ready(self):
        reply = "Program running: true"
        while not reply.startswith("Program running: false"):
            time.sleep(0.12)  # seconds
            self._send_ascii_bytes("running\n", self.dashboard_socket)
            reply = self._receive_ascii_bytes(self.dashboard_socket)
        assert reply.startswith("Program running: false"), reply

    def _move(self, q=np.zeros(6), v=None, a=None, t=None, r=None):
        # ensure that the robot is in the remote control mode
        self._ensure_ready()
        q_string = np.array2string(q, precision=6, separator=',').replace(" ", "")
        opts = []
        if a is not None:
            opts.append(f"a={a}")
        if v is not None:
            opts.append(f"v={v}")
        if t is not None:
            opts.append(f"t={t}")
        if r is not None:
            opts.append(f"r={r}")

        opts_string = ", ".join(opts)
        if len(opts) > 0:
            opts_string = ", " + opts_string
        return q_string, opts_string

    def _send_ctrl_cmd(self, program):
        self._send_ascii_bytes(program, self.controller_socket)
        self._ensure_ready()

    def movej(self, q=np.zeros(6), v=None, a=None, t=None, r=None):
        q_string, opts_string = self._move(q=q,v=v,a=a,t=t,r=r)
        program = "movej("+q_string+opts_string+")\n"
        self._send_ctrl_cmd(program)

    def movel(self, q=np.zeros(6), v=None, a=None, r=None):
        q_string, opts_string = self._move(q=q,v=v,a=a,r=r)
        program = "movel("+q_string+opts_string+")\n"
        self._send_ctrl_cmd(program)


    def movep(self, p=np.zeros(6),v=None, a=None, r=None):
        q_string, opts_string = self._move(q=p,v=v,a=a,r=r)
        program = "movep("+q_string+opts_string+")\n"
        self._send_ctrl_cmd(program)

    def screw_driving(self, f, v_limit):
        """
        f:          The amount of force the robot will exert along the TCP Z-axis (Newtons).
        v_limit:    Maximum TCP velocity along the Z axis (m/s).
        """
        self._ensure_ready()
        program = "screw_driving(" + str(f) + "," + str(v_limit) + ")\n"
        self._send_ctrl_cmd(program)


    def end_screw_driving(self):
        self._ensure_ready()
        program = "end_screw_driving()\n"
        self._send_ctrl_cmd(program)

    def zero_ftsensor(self):
        self._ensure_ready()
        program = "zero_ftsensor()\n"
        self._send_ctrl_cmd(program)

    def sleep(self, t):
        self._ensure_ready()
        program = "sleep(" + str(t) + ")\n"
        self._send_ctrl_cmd(program)


    def load_program(self, program_name):
        self._send_ascii_bytes(f"load {program_name}\n", self.dashboard_socket)
        loaded_program_response = self.dashboard_socket.recv(1024).decode()
        assert "File not found" not in loaded_program_response, f"The program file {program_name} was not found, received: {loaded_program_response}"
        assert "Error loading program" not in loaded_program_response, f"An error occurred while attempting to load the program {program_name}, received: {loaded_program_response}"
        loaded_program = self.get_loaded_program()
        assert program_name in loaded_program, f"The loaded program is incorrect. Expected {program_name}, but got {loaded_program}"
        return loaded_program_response.startswith("Loading program: ")

    def get_loaded_program(self):
        self._send_ascii_bytes(f"get loaded program\n", self.dashboard_socket)
        loaded_program = self.dashboard_socket.recv(1024).decode()
        return loaded_program


    def play_program(self):
        self._send_ascii_bytes("play\n", self.dashboard_socket)
        msg = self.dashboard_socket.recv(1024).decode()
        if not msg.startswith("Starting program"):
            print(f"{msg}Make sure that the robot is positioned in the starting position of the program or auto-initialize as described in: https://www.zacobria.com/universal-robots-knowledge-base-tech-support-forum-hints-tips/knowledge-base/auto-load-program/")
        return msg.startswith("Starting program")

    def stop_program(self):
        self._send_ascii_bytes("stop\n", self.dashboard_socket)
        return self.dashboard_socket.recv(1024).decode().startswith("Stopped")

    def record_samples(self, config_file='resources/record_configuration.xml', filename='robotdata.csv', overwrite=False,
                       frequency=500, samples=10, rtde_port=30004, RtdeConstructor=rtde.RTDE):
        assert samples > 0, "Expected positive number of samples. Use start_recording for samples=0."
        self._start_recording(config_file, filename, overwrite, frequency, samples, rtde_port, RtdeConstructor)
        self.stop_recording()

    def _start_recording(self, config_file, filename, overwrite,
                        frequency, samples, rtde_port, RtdeConstructor, publish_topic, rmq_client):
        assert self.recording_thread is None, "Already logging."

        # check for overwrite
        if os.path.isfile(filename) and not overwrite:
            _log.error(
                f"Output file {filename} already exists. If you wish to overwrite, use that parameter. Exiting...")
            sys.exit(1)

        connection = RtdeConstructor(self.ip_adr, rtde_port)

        self.recording_thread = threading.Thread(target=_recording_thread,
                                                 args=
                                                 [
                                                     connection,
                                                     config_file,
                                                     filename,
                                                     frequency,
                                                     samples,
                                                     self.recording_thread_queue,
                                                     publish_topic,
                                                     rmq_client
                                                 ]
                                                 )
        self.recording_thread.start()

    def start_recording(self, config_file='resources/record_configuration.xml', filename='robotdata.csv', overwrite=False,
                        frequency=500, rtde_port=30004, RtdeConstructor=rtde.RTDE, publish_topic=None, rmq_client=None):
        samples=0
        self._start_recording(config_file, filename, overwrite, frequency, samples, rtde_port, RtdeConstructor, publish_topic, rmq_client)

    def stop_recording(self):
        assert self.recording_thread is not None, "Should be recording data."

        _log.debug(f"Stop recording requested.")

        assert self.recording_thread_queue.empty(), "Expected this queue to be empty"
        self.recording_thread_queue.put(STOP_REQUEST)

        _log.debug(f"Waiting for thread to finish.")
        self.recording_thread.join()
        _log.debug(f"Thread finished.")
        self.recording_thread = None

        _stop_recording = False