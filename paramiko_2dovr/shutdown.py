#!/usr/bin/python3

import paramiko
import os
import time

user = "pi"
passwd = "ssr2"
command = "sudo shutdown -h now"

with open("host.txt") as f:
    s = f.readlines()

host_name = []
for i in range(len(s)):
    if s[i].startswith('#'): 
        pass
    else:
        host_name.append(s[i].replace("\n",''))
print(host_name)

for i in range(len(host_name)):
    target_host = host_name[i]
    with paramiko.SSHClient() as ssh:
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(target_host,username=user,password=passwd)
        stdin,stdout,stderr = ssh.exec_command(command)
