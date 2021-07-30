#!/usr/bin/python3

import paramiko
import os
import time

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
    with paramiko.SSHClient() as ssh:
        #初回ログイン時の対応
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        #ssh接続
        ssh.connect(host[i],username=user,password=passwd)
        #コマンド実行
        #stdin,stdout,stderr = ssh.exec_command(command)
        #stdin,stdout,stderr = ssh.exec_command(command,timeout=TIMEOUT)
        ssh.exec_command(command,timeout=TIMEOUT)

time.sleep(10) 
now = time.time()
start = now
while 1:
    print("\r %6.2f" % (now-start), end="" )
    time.sleep(0.5)
    now = time.time()






