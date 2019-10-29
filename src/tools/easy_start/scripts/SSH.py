import paramiko
import socket
import re


class Shell:
    def __init__(self, chan):
        self._shell = chan
        self._shell.settimeout(0.2)

    def exec_command(self, cmd):
        ret = ""
        try:
            self._shell.send(cmd+'\n')
            x = "  "
            while len(x) > 0:
                x =self._shell.recv(1)
                print(x.decode('utf-8', 'ignore'), end='')
                ret = ret + x.decode('utf-8', 'ignore')
            ret = self._del_color(ret)
            return ret
        except socket.timeout:
            ret = self._del_color(ret)
            return "socket timeout" if len(ret) == 0 else ret

    def _del_color(self, s):
        ret = ''
        l = len(s)
        i = 0
        while i < l :
            c = s[i]
            if c == '':
                while s[i] != 'm':
                    i = i+1
            else:
                ret = ret + c
            i = i + 1
        return ret
                

class SSH:
    def __init__(self, hostname, username, password):
        self._client = paramiko.SSHClient()
        self._client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.connected = False
        try:
            self._client.connect(hostname, username=username, password=password, timeout=1.0)
            self.connected = True
            self._sftp = self._client.open_sftp()
        except:
            raise Exception("connect to {}@{} failed".format(username, hostname))

    def close(self):
        self._client.close()

    def create_shell(self):
        if not self.connected:
            return None
        return Shell(self._client.invoke_shell())
    
    def transport_status(self, trans, total):
        print("\rtransport: {:.2f}%".format(float(trans)/total*100), end='')

    def upload(self, local, remote, callback=None):
        if not self.connected:
            return
        print('upload file from [{}] to [{}] \n'.format(local, remote))
        status = self.transport_status if callback is None else callback
        self._sftp.put(local, remote, status)
        print('\nupload file complete')

    def download(self, remote, local, callback=None):
        if not self.connected:
            return
        print('download file from [{}] to [{}] \n'.format(remote, local))
        status = self.transport_status if callback is None else callback
        self._sftp.get(remote, local, status)
        print('\ndownload file complete')
