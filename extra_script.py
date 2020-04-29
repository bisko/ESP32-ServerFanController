import datetime
import time
import os
from os.path import basename

from platformio import util

import requests
import sys

import re
import subprocess

Import("env")

today = datetime.datetime.today()

fw_suffix = today.strftime("%Y%m%d-%H%M%S")


# Fix path to ESP-OTA script.
esp_ota_path = "/Users/bisko/workspace/arduino/espota.py"

# List of module IDs
hw_modules = [
    "24:a:c4:26:71:c0",
    "24:a:c4:25:8d:dc"
]

def get_ip_for_hw(hw_addr):
    arp_out = subprocess.check_output(['arp', '-lan'])
    regexp = re.compile(r"\s*(\d+\.\d+\.\d+\.\d+)\s*(\S+)")
    hw_addresses = regexp.findall(str(arp_out))

    find_ip = [item for item in hw_addresses if hw_addr in item]

    if not find_ip:
        return False

    return find_ip[0][0]
    
def send_update_request(ip_addr, fw_path):
    print("Sending request to {0} to update to {1}".format(ip_addr, fw_path))

    # ESPOta way
    process_handle = subprocess.Popen([sys.executable, esp_ota_path,"-r", "-i", ip_addr, "--port", "8277", "-f", fw_path])

    return process_handle

def publish_firmware(source, target, env):
    print("Publishing")
    firmware_path = str(source[0])
    firmware_name = basename(firmware_path)
    new_path = "releases/{0}".format(firmware_name)

    print("Uploading {0} to modules.".format(firmware_name))

    os.rename(firmware_path, new_path)

    processes = []

    for mac_addr in hw_modules:
        module_ip = get_ip_for_hw(mac_addr)

        if not module_ip:
            print("Unable to find IP for {0}".format(mac_addr))
        else:
            processes.append(send_update_request(module_ip, new_path))

    print("Started send, waiting to finish")

    while True:
        running = False
        for process in processes:
            process.poll()
            if process.returncode is None:
                running = True

        if not running:
            break

    print("Upload done\n")

env.Replace(PROGNAME="fw-%s" % fw_suffix, UPLOADCMD=publish_firmware)