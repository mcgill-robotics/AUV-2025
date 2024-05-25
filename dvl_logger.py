#!/usr/bin/env python3
import serial
import time


def get_rpy(line):
    tokens = line.split(",")
    roll = float(tokens[6])
    pitch = float(tokens[7])
    yaw = float(tokens[8])

    return roll, pitch, yaw


def append_to_log(str):
    with open("dvllog.txt", "a+") as f:
        f.write(str)


def main():
    time_limit = 60 * 60  # seconds
    end_time = time.time() + time_limit
    conn = serial.Serial("/dev/dvl")
    # dvl's baud has been set to 115200 but its default is 9600.
    # There is a way to set the baudrate of the dvl through a command.
    conn.baudrate = 115200

    if not conn.isOpen():
        conn.open()

    conn.send_break()
    conn.flush()

    conn.write("wcr\r\n".encode("utf-8"))
    conn.flush()

    print("logging...")

    while conn.is_open and end_time > time.time():
        try:
            line = conn.readline().decode("utf-8")
            if line.startswith("wrp"):
                r, p, y = get_rpy(line)
                append_to_log(f"{r},{p},{y}\n")
        except Exception as e:
            print(e)


if __name__ == "__main__":
    append_to_log("r,p,y\n")
    try:
        main()
    except KeyboardInterrupt:
        exit()
