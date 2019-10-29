#!/usr/bin/env python3
#coding: utf-8

import os
import socket
from ssh2.session import Session
from ssh2.utils import wait_socket
from ssh2.fileinfo import FileInfo
import ssh2

class ssh_connection:
    def __init__(self, host, port, username, password):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect((host, port))
        self._session = Session()
        self._session.handshake(self._sock)
        self._session.userauth_password(username, password)
        self._session.set_blocking(True)

    def exec_command(self, command, rd=True, rt=False):
        channel = self._session.open_session()
        rtn = ''
        try:
            channel.execute(command)
            if rd:
                try:
                    size, data = channel.read()
                    while size > 0:
                        data = data.decode('utf-8').strip()
                        if rt:
                            rtn = rtn + data
                        else:
                            print(data)
                        size, data = channel.read()
                except: 
                    pass
                channel.wait_eof()
                size, data = channel.read()
                data = data.decode('utf-8')
                print(data.strip())
            channel.wait_closed
            channel.close() 
        except:
            pass
        return rtn

    def upload(self, local, remote):
        print(local)
        print(remote)
        fileinfo = os.stat(local)
        try:
            chan = self._session.scp_send64(remote, fileinfo.st_mode & 0o777, fileinfo.st_size, fileinfo.st_mtime, fileinfo.st_atime)
            with open(local, 'rb') as local_fh:
                l = 0
                for data in local_fh:
                    chan.write(data)
                    l = l+len(data)
                    print('\rupload                                        {0}%'\
                        .format(int(l/fileinfo.st_size*100)), end='', flush=True)
                print('\n')
        except ssh2.exceptions.SCPProtocolError as e:
            print(e)

    def download(self, remote, local):
        chan, fileinfo = self._session.scp_recv2(remote)
        total_size = fileinfo.st_size
        read_size = 0
        with open(local, 'wb') as local_fh:
            while read_size<total_size:
                size, data = chan.read()
                local_fh.write(data)
                read_size = read_size + size
                print('\rdownload                                        {0}%'\
                    .format(int(read_size/total_size*100)), end='', flush=True)
            print('\n')

    def close(self):
        self._session.disconnect()
        self._sock.close()