#!/usr/bin/env python3
#coding: utf-8

import os
import re

project_name = 'SEU-Kidsize'
username = 'root'
password = 'nvidia'
ssh_port = 22
remote_dir = '/home/nvidia/{}'.format(project_name)
start_up_file = '/etc/rc.local'
wan_file = '/etc/NetworkManager/system-connections/robocup'
lan_file = '/etc/NetworkManager/system-connections/static'
md5_file = 'md5.txt'
