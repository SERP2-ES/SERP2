#!/usr/bin/env python

import os
from bluetooth import *
from wifi import Cell, Scheme
import subprocess
import time

wpa_supplicant_conf = "/etc/wpa_supplicant/wpa_supplicant.conf"
wifi = []

def wifi_connect(ssid, psk):
    # write wifi config to fill
    cmd = 'wpa_passphrase "' + str(ssid) + '" "' + str(psk) + '" | sudo tee -a ' + str(wpa_supplicant_conf)

    #print(cmd)    
    os.system(cmd)

    #reconfigure wifi
    cmd = 'sudo wpa_cli -i wlan0 reconfigure'
    cmd_result = os.system(cmd)
    print(cmd + " - " + str(cmd_result))

    time.sleep(30)

def get_ip():
    cmd = 'hostname -I'
    cmd_result = subprocess.check_output(cmd, shell=True)
    ip = str(cmd_result).split()

    if not ip:
        return ""

    return str(ip[0])

def available_networks():
    Cells = Cell.all('wlan0')

    wifi_info = 'Found networks : \n'

    for current in range(len(Cells)):
        wifi_info +=  str(current) + "." + Cells[current].ssid + "\n"
        wifi.append(Cells[current].ssid)

    wifi_info += '\n'

    return wifi_info

try:
    while True:
        server_sock=BluetoothSocket( RFCOMM )
        server_sock.bind(("", 1))
        server_sock.listen(1)

        port = server_sock.getsockname()[1]

        uuid = "00001101-0000-1000-8000-00805F9B34FB"

        advertise_service( server_sock, "Bluetooth_terminal",
                service_id = uuid,
                service_classes = [ uuid, SERIAL_PORT_CLASS ],
                profiles = [ SERIAL_PORT_PROFILE ])


        print('Waiting for connection on RFCOMM channel 1')

        client_sock, client_info = server_sock.accept()
        print('Accepted connection from' +  str(client_info))

        client_sock.send('Connection stablished \nStarting initial configuration \n')

        while True:
            string = available_networks()

            client_sock.send(str(string))
            client_sock.send("Choose network number!\n")
            ssid = client_sock.recv(1024)

            if (int(ssid) > len(wifi) or int(ssid) < 0) :
                client_sock.send("Invalid network number!\n")	
                continue

            selected = str(wifi[int(ssid)])

            client_sock.send("Insert network password")
            password = client_sock.recv(1024)

            print('Received: ssid > ' + selected + ' pass > ' + str(password))

            client_sock.send("Connecting to network... \n")		
            wifi_connect(selected.strip(), password.strip())

            ip_address = get_ip()

            if ip_address == "":
                print('Error connecting to the network\n')
                continue
            
            break

        client_sock.send("IP address: " + ip_address + " \n")

        # finished config   
        print('Finished configuration\n')
        client_sock.send("Finished configuration\nDisconecting ...")
        client_sock.close()
        server_sock.close()
        break

except (KeyboardInterrupt, SystemExit):
    client_sock.close()
    server_sock.close()
    print('\nExiting\n')




