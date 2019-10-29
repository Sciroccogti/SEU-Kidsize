#!/usr/bin/env python3
#coding: utf-8

import common
import sys
import ssh_connection
import config


if __name__ == '__main__': 
    if not common.check_argv(sys.argv):
        common.print_error('no enough arguments')
        exit(2)

    robot_id = sys.argv[1]
    if not common.check_id(robot_id):
        common.print_error('please check the robot id')
        exit(3)

    args = common.parse_argv(sys.argv)

    ip_address = common.get_ip(robot_id)
    ssh_client = ssh_connection.ssh_connection(ip_address, config.ssh_port, config.username, config.password)

    cmd = "sed -i '/%s/d' %s; poweroff"%(config.exec_file_name, config.start_up_file)
    ssh_client.exec_command(cmd)