#!/usr/bin/env python3
#coding: utf-8

import common
import sys
import ssh_connection
import config


def number_of_1(n):
    return bin(int(n) & 0xffffffff).count('1')


if __name__ == '__main__': 
    if not common.check_argv(sys.argv, 7):
        common.print_error('no enough arguments')
        common.print_info('./set_wifi.py id ssid pswd ip netmask gateway')
        exit(2)

    robot_id = sys.argv[1]
    if not common.check_id(robot_id):
        common.print_error('please check the robot id')
        exit(3)

    ssid = sys.argv[2]
    pswd = sys.argv[3]
    ip = sys.argv[4]
    netmask_str = sys.argv[5]
    netmasks = netmask_str.split('.')
    netmask=0
    for n in netmasks:
        netmask = netmask+number_of_1(n)
    gateway = sys.argv[6]

    ip_address = common.get_ip(robot_id)

    cmd1 = '''
    sed -i '/ssid=/cssid={sid}' {file};
    sed -i '/psk=/cpsk={psk}' {file};
    sed -i '/address1=/caddress1={adr}' {file};
    '''.format(sid=ssid, psk=pswd, adr=addr, file=config.wan_file)
    cmd2 = '''
    sed -i '/address1=/caddress1={adr}' {file};
    '''.format(adr=addr, file=config.lan_file)

    ssh_client = ssh_connection.ssh_connection(ip_address, config.ssh_port, config.username, config.password)
    ssh_client.exec_command(cmd1)
    ssh_client.exec_command(cmd2)
    ssh_client.exec_command('poweroff')