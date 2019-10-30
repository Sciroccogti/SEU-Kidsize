#!/usr/bin/env python3
#coding: utf-8

import sys
import os
import common
import config
from PyQt5 import QtWidgets, QtCore
import SSH

class TerminalWidget(QtWidgets.QWidget):
    def __init__(self, shell):
        super().__init__()
        self._shell = shell
        self._outputBox = QtWidgets.QTextEdit()
        self._inputLine = QtWidgets.QLineEdit()
        self._outputBox.setReadOnly(True)
        self._inputLine.setPlaceholderText('input command here, enter to execute')
        self._inputLine.returnPressed.connect(self._run)
        mainLayout = QtWidgets.QVBoxLayout()
        mainLayout.addWidget(self._outputBox)
        mainLayout.addWidget(self._inputLine)
        self.setLayout(mainLayout)
    
    def _run(self):
        cmd = self._inputLine.text()
        if len(cmd) == 0:
            return
        self._inputLine.setText('')
        if cmd == 'clear':
            self._outputBox.setText('')
        else:
            ret = self._shell.exec_command(cmd)
            self._outputBox.append(ret)


class EasyWindow(QtWidgets.QMainWindow):
    closed  = QtCore.pyqtSignal(str)
    def __init__(self, mid):
        super().__init__()
        self._id = mid
        self._host = common.get_config(config.conf_file, 'players.{}.address'.format(self._id))
        self._ssh = None
        self._connected = False
        self._shell = None
        try:
            self._ssh = SSH.SSH(self._host, config.username, config.password)
            self._shell = self._ssh.create_shell()
            self._connected = True
        except Exception as e:
            print(e.args)
        self._init_ui()
        if self._connected:
            self._statusBar.showMessage('connected to {}@{}'.format(config.username, self._host))
        else:
            self._statusBar.showMessage('connected to {}@{} failed'.format(config.username, self._host))
            

    def _init_ui(self):                
        self.setWindowTitle('EasyStart: {}'.format(self._id))
        mainWidget = QtWidgets.QWidget()
        mainLayout = QtWidgets.QVBoxLayout()
        files = os.listdir(config.local_root+'/src')
        srcGroup = QtWidgets.QGroupBox("Source Files")
        gboxLayout = QtWidgets.QVBoxLayout()
        self._srcCheckBoxes = []
        for f in files:
            if not os.path.isdir(config.local_root+'/src/'+f):
                continue
            box = QtWidgets.QCheckBox(f)
            self._srcCheckBoxes.append(box)
            gboxLayout.addWidget(box)
        srcGroup.setLayout(gboxLayout)
        leftLayout = QtWidgets.QVBoxLayout()
        leftLayout.addWidget(srcGroup)

        self._uploadLocalLine = QtWidgets.QLineEdit()
        self._uploadLocalLine.setPlaceholderText('local path')
        self._ulRealCheck = QtWidgets.QCheckBox('realpath')
        self._uploadRemoteLine = QtWidgets.QLineEdit()
        self._uploadRemoteLine.setPlaceholderText('remote path')
        self._urRealCheck = QtWidgets.QCheckBox('realpath')
        self._uploadBtn = QtWidgets.QPushButton('Upload')
        self._uploadBtn.clicked.connect(self.proc_btn_upload)
        uploadLayout = QtWidgets.QHBoxLayout()
        uploadLayout.addWidget(self._uploadLocalLine)
        uploadLayout.addWidget(self._ulRealCheck)
        uploadLayout.addWidget(self._uploadRemoteLine)
        uploadLayout.addWidget(self._urRealCheck)
        uploadLayout.addWidget(self._uploadBtn)

        self._downloadRemoteLine = QtWidgets.QLineEdit()
        self._downloadRemoteLine.setPlaceholderText('remote path')
        self._drRealCheck = QtWidgets.QCheckBox('realpath')
        self._downloadLocalLine = QtWidgets.QLineEdit()
        self._downloadLocalLine.setPlaceholderText('local path')
        self._dlRealCheck = QtWidgets.QCheckBox('realpath')
        self._downloadBtn = QtWidgets.QPushButton('Download')
        self._downloadBtn.clicked.connect(self.proc_btn_download)
        downloadLayout = QtWidgets.QHBoxLayout()
        downloadLayout.addWidget(self._downloadRemoteLine)
        downloadLayout.addWidget(self._drRealCheck)
        downloadLayout.addWidget(self._downloadLocalLine)
        downloadLayout.addWidget(self._dlRealCheck)
        downloadLayout.addWidget(self._downloadBtn)

        self._inputBox = QtWidgets.QTextEdit()
        self._inputBox.setAcceptRichText(False)
        self._tabWidget = QtWidgets.QTabWidget()
        rightLayout = QtWidgets.QVBoxLayout()
        rightLayout.addLayout(uploadLayout)
        rightLayout.addLayout(downloadLayout)
        rightLayout.addWidget(self._tabWidget)
        addTermBtn = QtWidgets.QPushButton("New Terminal")
        addTermBtn.clicked.connect(self.proc_btn_add_term)
        rmTermBtn = QtWidgets.QPushButton("Remove Terminal")
        rmTermBtn.clicked.connect(self.proc_btn_rm_term)
        termLayout = QtWidgets.QHBoxLayout()
        termLayout.addWidget(addTermBtn)
        termLayout.addWidget(rmTermBtn)
        rightLayout.addLayout(termLayout)
        
        upLayout = QtWidgets.QHBoxLayout()
        upLayout.addLayout(leftLayout)
        upLayout.addLayout(rightLayout)
        mainLayout.addLayout(upLayout)
        self._outputBox = QtWidgets.QTextEdit()
        self._outputBox.setReadOnly(True)
        mainLayout.addWidget(self._outputBox)
        mainWidget.setLayout(mainLayout)
        self.setCentralWidget(mainWidget)
        self._statusBar = QtWidgets.QStatusBar()
        self.setStatusBar(self._statusBar)

    def proc_btn_connect(self):
        if not self._connected:
            try:
                self._ssh = SSH.SSH(self._host, config.username, config.password)
                self._shell = self._ssh.create_shell()
                self._connected = True
                self._connBtn.setText('Disconnect')
                self._statusBar.showMessage('connected to {}@{}'.format(config.username, self._host))
                self._uploadBtn.setEnabled(True)
                self._downloadBtn.setEnabled(True)
            except Exception as e:
                self._statusBar.showMessage(e.args[0])
        else:
            self._ssh.close()
            self._connected = False
            self._connBtn.setText('Connect')
            self._statusBar.showMessage('')
            self._ssh = None
            self._shell = None
            self._uploadBtn.setEnabled(False)
            self._downloadBtn.setEnabled(False)

    def proc_btn_add_term(self):
        if not self._connected:
            return
        terminal = TerminalWidget(self._ssh.create_shell())
        self._tabWidget.addTab(terminal, "Terminal")
        self._tabWidget.setCurrentIndex(self._tabWidget.count()-1)

    def proc_btn_rm_term(self):
        self._tabWidget.removeTab(self._tabWidget.currentIndex())

    def transport_status(self, trans, total):
        self._statusBar.showMessage("transport: {:.2f}%".format(float(trans)/total*100))
        QtWidgets.QApplication.processEvents()

    def proc_btn_upload(self):
        if self._ssh is None:
            return
        local = self._uploadLocalLine.text()
        if not self._ulRealCheck.isChecked():
            local = config.local_root + '/' + local
        remote = self._uploadRemoteLine.text()
        if not self._urRealCheck.isChecked():
            remote = config.remote_root + '/' + remote
        self._outputBox.append('upload file from [{}] to [{}] begin'.format(local, remote))
        self._ssh.upload(local, remote, self.transport_status)
        self._outputBox.append('upload file from [{}] to [{}] complete '.format(local, remote))
        self._statusBar.showMessage('connected to {}@{}'.format(config.username, self._host))

    def proc_btn_download(self):
        if self._ssh is None:
            return
        local = self._downloadLocalLine.text()
        if not self._dlRealCheck.isChecked():
            local = config.local_root + '/' + local
        remote = self._downloadRemoteLine.text()
        if not self._drRealCheck.isChecked():
            remote = config.remote_root + '/' + remote
        self._outputBox.append('download file from [{}] to [{}] begin'.format(remote, local))
        self._ssh.download(remote, local, self.transport_status)
        self._outputBox.append('download file from [{}] to [{}] complete'.format(remote, local))
        self._statusBar.showMessage('connected to {}@{}'.format(config.username, self._host))
        
    def closeEvent(self, event):
        if not self._ssh is None:
            self._ssh.close()
        self.closed.emit(self._id)
        
class StartDlg(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self._idSpin = QtWidgets.QSpinBox()
        self._idSpin.setRange(0, 5)
        startBtn = QtWidgets.QPushButton('Create New')
        startBtn.clicked.connect(self.start_new)
        mainLayout = QtWidgets.QVBoxLayout()
        mainLayout.addWidget(self._idSpin)
        mainLayout.addWidget(startBtn)
        mainWidget = QtWidgets.QWidget()
        mainWidget.setLayout(mainLayout)
        self.setCentralWidget(mainWidget)
        self.windows = {}

    def close_window(self, idx):
        pass
        #if idx in self.windows.keys():
        #    self.windows.pop(idx)

    def start_new(self):
        idx = self._idSpin.text()
        self.windows[idx] = EasyWindow(idx)
        self.windows[idx].closed.connect(self.close_window)
        self.windows[idx].showNormal()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    dlg = StartDlg()
    dlg.show()
    sys.exit(app.exec_())