#!/usr/bin/env python3
import time
import serial

PORT = "/dev/ttyUSB0"
BAUDRATE = 921600   # change this if your unit is currently configured differently
SAVE_TO_FLASH = True  # set True if you want the change to persist after power cycle

def send_cmd(ser, cmd, wait=0.3):
    ser.write((cmd + "\r").encode("ascii"))
    ser.flush()
    time.sleep(wait)
    data = ser.read_all().decode("latin1", errors="replace")
    print(f">>> {cmd}")
    print(data)

with serial.Serial(
    port=PORT,
    baudrate=BAUDRATE,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.2,
) as ser:
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    send_cmd(ser, "SERVICEMODE", wait=1.0)
    send_cmd(ser, "i m")
    send_cmd(ser, "m 1")   # 0 = 125 samples/s, 1 = 250 samples/s, 2 = 500 samples/s, 3 = 1000 samples/s, 4 = 2000 samples/s, 5 = external trigger
    send_cmd(ser, "i m")

    if SAVE_TO_FLASH:
        send_cmd(ser, "s", wait=0.3)
        send_cmd(ser, "Y", wait=1.0)

    send_cmd(ser, "x N", wait=0.5)  # immediate exit to Normal Mode