#!/usr/bin/python3

import paramiko
import os
import datetime as dt
import scp

user = "pi"
passwd = "ssr2"
command = 'python3 /home/pi/2DOVR/2dovr.py'

with open("host.txt") as f:
    s = f.readlines()

host_name = []
for i in range(len(s)):
    if s[i].startswith('#'): 
        pass
    else:
        host_name.append(s[i].replace("\n",''))
print(host_name)

path = "/home/pi/2DOVR/result/"
date = dt.datetime.now()
date = str(date.strftime('%Y%m%d'))
for i in range(len(host_name)):
    target_host = host_name[i]
    file_name = path +target_host + "-" + date + "*.txt"
    with paramiko.SSHClient() as ssh:
        # 初回ログイン時
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        # ssh接続する
        ssh.connect(target_host, username=user, password=passwd)

        print('[SCP転送開始]')
        #ssh_connect = ssh.open_sftp()
        #ssh_connect.get(file_name,'/home/yamada/m2/2dovr_test/')
        # scp clientオブジェクト生成
        with scp.SCPClient(ssh.get_transport(), sanitize=lambda x: x) as scp:
            # scp送信する場合はput
            #scp.put('test.csv', '/var/samba/.')

            # scp受信する場合はget
            scp.get(file_name)





