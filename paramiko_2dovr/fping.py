#!/usr/bin/python3

import paramiko
import os
import time
import subprocess as sp

user = "pi"
passwd = "ssr2"
command = 'python3 /home/pi/2DOVR/2dovr.py'
TIMEOUT=1000

with open("host.txt") as f:
    host_array = f.readlines()

host = []
for i in range(len(host_array)):
    if host_array[i].startswith('#'): 
        pass
    else:
        
        host.append(host_array[i].replace("\n",''))
print(host)

for i in range(len(host)):
    command = 'fping ' + host[i]
    #print(command)
    sp.Popen(command.split())
    

