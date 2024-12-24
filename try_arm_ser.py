import serial
import time
import argparse
import threading

def read_serial():
    while True:
        if ser.in_waiting > 0:
            print(ser.in_waiting)
            with lock:
                data = ser.readline().decode('utf-8')
                if data:
                    print(f"Received: {data}", end='')
        time.sleep(0.02)

def main():
    global ser
    global lock
    lock = threading.Lock()
    parser = argparse.ArgumentParser(description='Serial JSON Communication')
    parser.add_argument('port', type=str, help='Serial port name (e.g., COM1 or /dev/ttyUSB0)')

    args = parser.parse_args()

    ser = serial.Serial(args.port, baudrate=115200, dsrdtr=None)
    time.sleep(2)
    ser.setRTS(False)
    ser.setDTR(False)
    time.sleep(2)

    serial_recv_thread = threading.Thread(target=read_serial)
    serial_recv_thread.daemon = True
    serial_recv_thread.start()

    try:
        while True:
            command = input("")
            with lock:
                ser.write(command.encode() + b'\n')

            # ser.write('{"T":"105"}'.encode() + b'\n')
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
