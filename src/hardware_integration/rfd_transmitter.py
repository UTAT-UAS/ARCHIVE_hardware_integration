#!/usr/bin/python3

from __future__ import annotations
import rospy
from std_msgs.msg import String
import serial
import time

import os

START_MESSAGE = "BEGIN FLIGHT"

# Base implementation, class based version of old FS rfd code
# Untested

# TODO:
# need transmission schema
# integration into flight stack


class GCS:
    def __init__(self) -> None:
        self.count = 0
        self.rfd = RFD()

        # Set up the serial connection
        self.pub = rospy.Publisher("/aeac/TODO", String, queue_size=10)  # TODO: where to send rfd messages?
        rospy.init_node('ground_control_station')

        self.send_rate = rospy.Rate(10)  # Send messages at 0.5 Hz
        self.listen_rate = rospy.Rate(100)

    def begin(self) -> None:
        self.begin_flight()
        self.recieve_data()

    def begin_flight(self) -> None:
        # send START_MESSAGE 5 times
        while not rospy.is_shutdown():
            if self.count < 5:
                self.rfd.transmit(START_MESSAGE)
                self.send_rate.sleep()
                self.count += 1
        # TODO: wait for confirmation response

    def recieve_data(self) -> None:
        while not rospy.is_shutdown():
            msg = self.rfd.listen()
            if msg:
                print(msg)  # TODO: publish message
            self.listen_rate.sleep()


class GCSLink:
    def __init__(self) -> None:
        self.rfd = RFD()

        # Set up the serial connection
        self.pub = rospy.Publisher("/aeac/TODO", String, queue_size=10)  # TODO: where to send rfd messages?
        self.pub = rospy.Subscriber("/aeac/TODO", String)  # TODO: how to wait for message to send via rfd? use service?
        rospy.init_node('ground_control_link')

        self.send_rate = rospy.Rate(10)  # Send messages at 0.5 Hz
        self.listen_rate = rospy.Rate(100)

    def wait_for_start(self) -> None:
        while not rospy.is_shutdown():
            msg = self.rfd.listen()
            if msg == START_MESSAGE:
                return
            self.listen_rate.sleep()


class RFD:
    def __init__(self) -> None:
        self.port = '/dev/ttyUSB0'  # Replace with your device's serial port
        self.baud_rate = 115200  # Replace with your device's baud rate
        self.end = '\0'

    def listen(self) -> str | None:
        ser = serial.Serial(self.port, self.baud_rate)

        try:
            message = ser.read_until('\0'.encode()).decode()
        except UnicodeDecodeError:
            ser.close()
            return None

        ser.close()
        return message

    def transmit(self, msg: String) -> None:
        msg = str(msg + self.end)
        bbytes = msg.encode()

        rfd_talker = serial.Serial(self.port, self.baud_rate, write_timeout=0)
        rfd_talker.write(bbytes)
        rfd_talker.close()
