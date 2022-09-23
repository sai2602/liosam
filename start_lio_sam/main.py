# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import os
from time import sleep
from threading import Thread


liosam_dir = ""# "/home/sai/LIO_SAM"

def update_working_dir():
    global liosam_dir
    liosam_dir = os.getcwd()
    liosam_dir = liosam_dir.rsplit("/", 1)[0]


def change_working_dir():
    os.chdir(liosam_dir)
    print("Changed current working directory")


def list_files():
    change_working_dir()
    files = [f for f in os.listdir('.') if os.path.isfile(f)]

    for each_file in files:
        print(each_file)


def run_imu_calib():
    print("Applying imu calibration")
    change_working_dir()
    os.system("./run_imu_calib.sh")


def run_imu_converter():
    print("Converting imu values")
    change_working_dir()
    os.system("./run_imu_converter.sh")


def run_vlp16_launcher():
    print("Launching velodyne data converter")
    change_working_dir()
    os.system("./run_vlp16.sh")


def run_lio_sam():
    change_working_dir()
    print("Launching map generator")
    os.system("./run_lio_sam.sh")


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    update_working_dir()

    imu_calib_thread = Thread(target=run_imu_calib)
    imu_converter_thread = Thread(target=run_imu_converter)
    vlp16_launcher_thread = Thread(target=run_vlp16_launcher)
    liosam_launcher_thread = Thread(target=run_lio_sam)
    #
    imu_calib_thread.setDaemon(True)
    imu_converter_thread.setDaemon(True)
    vlp16_launcher_thread.setDaemon(True)
    liosam_launcher_thread.setDaemon(True)
    #
    imu_calib_thread.start()
    sleep(1)
    #
    imu_converter_thread.start()
    sleep(1)
    #
    vlp16_launcher_thread.start()
    sleep(5)
    #
    liosam_launcher_thread.start()

    # run_lio_sam()

    while True:
        pass

    # list_files()

