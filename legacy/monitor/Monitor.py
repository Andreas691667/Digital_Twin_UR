#!/usr/bin/env python
# Copyright (c) 2020-2022, Universal Robots A/S,
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Universal Robots A/S nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL UNIVERSAL ROBOTS A/S BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import sys
import threading
import argparse
import logging
import json

sys.path.append("..")

from config.rmq_config import RMQ_CONFIG
from config.robot_config import ROBOT_CONFIG
from rmq.RMQClient import Client
import ur_rtde_client_lib.rtde as rtde
import ur_rtde_client_lib.rtde_config as rtde_config
import ur_rtde_client_lib.csv_writer as csv_writer
import ur_rtde_client_lib.csv_binary_writer as csv_binary_writer


class Monitor:
    def __init__(self) -> None:
        self.args = None
        self.output_names, self.output_types = self.get_input_argument()

        # Instantiate RMQ_CLIENT
        self.rmq_client = Client(RMQ_CONFIG.RMQ_SERVER_IP, RMQ_CONFIG.RMQ_SERVER_PORT)

        

        # Setup connection to robot
        # makes con, that can be accessed via self.con
        self.rdte_connect()

        # get controller version
        self.rtde_con.get_controller_version()

        # setup recipies
        self.setup_recipes()

        # start data synchronization
        self.start_data_synchronization()

        self.monitor_thread = threading.Thread(target=self.start_recording)
        self.monitor_event = threading.Event()

        self.configure_rmq_client()

        self.monitor_thread.start()

    def configure_rmq_client(self) -> None:
        self.rmq_client.configure_outgoing_channel(
            RMQ_CONFIG.MONITOR_EXCHANGE, RMQ_CONFIG.FANOUT
        )

    def rdte_connect(self) -> None:
        # Instantiate RTDE connection
        self.rtde_con = rtde.RTDE(ROBOT_CONFIG.ROBOT_HOST, ROBOT_CONFIG.ROBOT_PORT_OUT)
        self.rtde_con.connect()

    # TODO: add return type, I think it is (str, str)
    def get_input_argument(self):
        # ---- Get arguments ----
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--host", default="localhost", help="name of host to connect to (localhost)"
        )
        parser.add_argument(
            "--port", type=int, default=30004, help="port number (30004)"
        )
        parser.add_argument(
            "--samples", type=int, default=0, help="number of samples to record"
        )
        parser.add_argument(
            "--frequency", type=int, default=12, help="the sampling frequency in Herz"
        )
        parser.add_argument(
            "--config",
            default="record_configuration.xml",
            help="data configuration file to use (record_configuration.xml)",
        )
        parser.add_argument(
            "--output",
            default="robot_data.csv",
            help="data output file to write to (robot_data.csv)",
        )
        parser.add_argument(
            "--verbose", help="increase output verbosity", action="store_true"
        )
        parser.add_argument(
            "--buffered",
            help="Use buffered receive which doesn't skip data",
            action="store_true",
        )
        parser.add_argument(
            "--binary", help="save the data in binary format", action="store_true"
        )
        self.args = parser.parse_args()

        if self.args.verbose:
            logging.basicConfig(level=logging.INFO)

        conf = rtde_config.ConfigFile(self.args.config)
        output_names, output_types = conf.get_recipe("out")

        return output_names, output_types

    def setup_recipes(self) -> None:
        # setup recipes
        if not self.rtde_con.send_output_setup(
            self.output_names, self.output_types, frequency=self.args.frequency
        ):
            logging.error("Unable to configure output")
            sys.exit()

    def start_data_synchronization(self) -> None:
        # start data synchronization
        if not self.rtde_con.send_start():
            logging.error("Unable to start synchronization")
            sys.exit()

    def start_recording(self) -> None:
        writeModes = "wb" if self.args.binary else "w"
        with open(self.args.output, writeModes) as csvfile:
            writer = None

            if self.args.binary:
                writer = csv_binary_writer.CSVBinaryWriter(
                    csvfile, self.output_names, self.output_types
                )
            else:
                writer = csv_writer.CSVWriter(
                    csvfile, self.output_names, self.output_types
                )

            writer.writeheader()

            # Loop with required frequency
            i = 1
            while not self.monitor_event.is_set():
                if i % self.args.frequency == 0:
                    if self.args.samples > 0:
                        sys.stdout.write("\r")
                        sys.stdout.write(
                            "{:.2%} done.".format(float(i) / float(self.args.samples))
                        )
                        sys.stdout.flush()
                    else:
                        sys.stdout.write("\r")
                        sys.stdout.write("{:3d} samples.".format(i))
                        sys.stdout.flush()
                if self.args.samples > 0 and i >= self.args.samples:
                    self.monitor_event.clear()

                try:
                    if self.args.buffered:
                        state = self.rtde_con.receive_buffered(self.args.binary)
                    else:
                        state = self.rtde_con.receive(self.args.binary)

                    # Check if there is a reading
                    if state is not None:
                        # Write to a file and return data in a list
                        data = writer.writerow(state)
                        
                        # Send state over RMQ
                        self.rmq_client.send_message(
                            data, RMQ_CONFIG.MONITOR_EXCHANGE
                        )

                        i += 1

                except rtde.RTDEException:
                    self.rtde_con.disconnect()
                    sys.exit()

        sys.stdout.write("\rComplete!            \n")
        self.rtde_con.send_pause()
        self.rtde_con.disconnect()

    def stop_recording(self) -> None:
        self.monitor_event.set()
