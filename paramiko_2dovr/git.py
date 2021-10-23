#!/usr/bin/python3

import paramiko
import os
import datetime as dt
import scp

user = "pi"
passwd = "ssr2"
command2 = 'rm /home/pi/2DOVR/result/*.txt'

print("git pullコマンドを使いますか？(default=y)")
print("yes: git -C /home/pi/2DOVR pull")
print("no: git reset --hard origin/yamada")

ch = str(input("y or n ？"))
if ch == "":
    ch = 'y'
if ch == 'y':
    command = 'git -C /home/pi/2DOVR pull'
else:
    command = 'git reset --hard origin/yamada'
with open("host.txt") as f:
    s = f.readlines()

host = []
for i in range(len(s)):
    if s[i].startswith('#'): 
        pass
    else:
        host.append(s[i].replace("\n",''))
print(host)

for i in range(len(host)):
    with paramiko.SSHClient() as ssh:
        #初回ログイン時の対応
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        #ssh接続
        ssh.connect(host[i],username=user,password=passwd)
         #コマンド実行
        stdin,stdout,stderr = ssh.exec_command(command)
        print("[" + host[i] + "]")
        print("[stdo]--->標準出力 , [err]--->エラー出力\n")
        for o in stdout:
            print('[stdo]',o,end='')
        for e in stderr:
            print('[err]',e,end='')
