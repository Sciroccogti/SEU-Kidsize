#!/usr/bin/env python3
#coding: utf-8

import common
import sys
import ssh_connection
import config
import datetime


if __name__ == '__main__': 
    if not common.check_argv(sys.argv):
        common.print_error('no enough arguments')
        exit(2)

    robot_id = sys.argv[1]
    if not common.check_id(robot_id):
        common.print_error('please check the robot id')
        exit(3)

    if not common.build_project(True):
        common.print_error('build error, please check code')
        exit(4)

    args = common.parse_argv(sys.argv)
    ip_address = common.get_ip(robot_id)

    if not common.check_net(ip_address):
        common.print_error('can not connect to host, please check network')
        exit(6)
        
    ssh_client = ssh_connection.ssh_connection(ip_address, config.ssh_port, config.username, config.password)

    if not common.compress_files(ssh_client):
        common.print_error('compress files error, please check')
        exit(5)
        
    ssh_client.upload(config.project_dir+'/bin/'+config.compress_file_name, config.remote_dir+config.compress_file_name)

    cmd = "cd %s; tar zxmf %s; sed -i '$acd %s' %s; sed -i '$a./%s %s &' %s; poweroff;"%(config.remote_dir, config.compress_file_name,\
        config.remote_dir+config.target_dir, config.start_up_file, config.exec_file_name, args, config.start_up_file)
    ssh_client.exec_command(cmd)