# -*- coding: utf-8 -*-
"""
This module contains functions for serial communication with ESP32 MicroPython.
relating to file system based operations.

You may:

* ls - list files on the device. Based on the equivalent Unix command.
* rm - remove a named file on the device. Based on the Unix command.
* put - copy a named local file onto the device a la equivalent FTP command.
* get - copy a named file from the device to the local file system a la FTP.
* tree - get the device folder structure.
"""
from __future__ import print_function
import ast
import argparse
import sys
import os
import time
import os.path
from PyQt5.QtSerialPort import QSerialPort
from PyQt5.QtCore import QIODevice

__all__ = ["ls", "rm", "put", "get", "get_serial", "tree"]


def open_serial(port):
    """
    Creates a new serial link instance.
    """
    serial = QSerialPort()
    serial.setPortName(port)
    if serial.open(QIODevice.ReadWrite):
        serial.setDataTerminalReady(True)
        if not serial.isDataTerminalReady():
            # Using pyserial as a 'hack' to open the port and set DTR
            # as QtSerial does not seem to work on some Windows :(
            # See issues #281 and #302 for details.
            serial.close()
            pyser = serial.Serial(port)  # open serial port w/pyserial
            pyser.dtr = True
            pyser.close()
            serial.open(QIODevice.ReadWrite)
        serial.setBaudRate(115200)
    else:
        msg = _("Cannot connect to device on port {}").format(port)
        raise IOError(msg)

    return serial

def close_serial(serial):
    """
    Close and clean up the currently open serial link.
    """
    if serial:
        serial.close()

def read_until(serial, token, timeout=5000):
    buff = bytearray()
    while True:
        if not (serial.waitForReadyRead(timeout)):
            raise TimeoutError(_('Transfer synchronization processing failed'))
        data = bytes(serial.readAll())  # get all the available bytes.
        buff.extend(data)
        if token in buff:
            break
    return buff

def flush_to_msg(serial, token):
    read_until(serial, token)

def raw_on(serial):
    """
    Puts the device into raw mode.
    """

    raw_repl_msg = b"raw REPL; CTRL-B to exit\r\n>"
    # Send CTRL-B to end raw mode if required.
    serial.write(b"\x02")
    # Send CTRL-C three times between pauses to break out of loop.
    for i in range(3):
        serial.write(b"\r\x03")
        time.sleep(0.01)
    serial.flush()
    # Go into raw mode with CTRL-A.
    serial.write(b"\r\x01")
    flush_to_msg(serial, raw_repl_msg)
    # Soft Reset with CTRL-D
    serial.write(b"\x04")
    flush_to_msg(serial, b"soft reboot\r\n")
    # Some MicroPython versions/ports/forks provide a different message after
    # a Soft Reset, check if we are in raw REPL, if not send a CTRL-A again
    data = read_until(serial, raw_repl_msg)
    if not data.endswith(raw_repl_msg):
        serial.write(b"\r\x01")
        flush_to_msg(serial, raw_repl_msg)
    serial.flush()


def raw_off(serial):
    """
    Takes the device out of raw mode.
    """
    serial.write(b"\x02")  # Send CTRL-B to get out of raw mode.

def send_cmd_blocking(serial, commands):
    """
    Separated RAW REPL ON / OFF processing from execute function to trace device
    directory hierarchy.   
    """
    result = b""
    for command in commands:
        command_bytes = command.encode("utf-8")
        for i in range(0, len(command_bytes), 32):
            serial.write(command_bytes[i : min(i + 32, len(command_bytes))])
            time.sleep(0.01)
        serial.write(b"\x04")
        response = read_until(serial, b"\x04>")  # Read until prompt.
        out, err = response[2:-2].split(b"\x04", 1)  # Split stdout, stderr
        result += out
        if err:
            return b"", err

    return result, err

def send_cmd(serial, commands, cb):
    """
    Separated RAW REPL ON / OFF processing from execute function to trace device
    directory hierarchy.   
    """

    serial.readyRead.connect(cb)

    result = b""
    for command in commands:
        command_bytes = command.encode("utf-8")
        for i in range(0, len(command_bytes), 32):
            serial.write(command_bytes[i : min(i + 32, len(command_bytes))])
            time.sleep(0.01)
        serial.write(b"\x04")

