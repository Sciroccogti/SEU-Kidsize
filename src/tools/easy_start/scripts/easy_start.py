#!/usr/bin/env python3
#coding: utf-8

import sys
import os
import common
from PyQt5 import QtWidgets
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
    def __init__(self):
        super().__init__()
        mypath = os.path.realpath(__file__)
        self._root = mypath[:mypath.find('/src/tools')]
        self._init_ui()
        self._ssh = None
        self._connected = False
        self._shell = None

    def _init_ui(self):
        self.setWindowTitle('EasyStart')
        mainWidget = QtWidgets.QWidget()
        mainLayout = QtWidgets.QVBoxLayout()
        self._idSpin = QtWidgets.QSpinBox()
        self._idSpin.setRange(0, 5)
        files = os.listdir(self._root+'/src')
        srcGroup = QtWidgets.QGroupBox("Source Files")
        gboxLayout = QtWidgets.QVBoxLayout()
        self._srcCheckBoxes = []
        for f in files:
            if not os.path.isdir(self._root+'/src/'+f):
                continue
            box = QtWidgets.QCheckBox(f)
            self._srcCheckBoxes.append(box)
            gboxLayout.addWidget(box)
        srcGroup.setLayout(gboxLayout)
        idLayout = QtWidgets.QHBoxLayout()
        idLayout.addWidget(QtWidgets.QLabel('Player ID:'))
        idLayout.addWidget(self._idSpin)
        leftLayout = QtWidgets.QVBoxLayout()
        leftLayout.addLayout(idLayout)
        leftLayout.addWidget(srcGroup)

        self._connBtn = QtWidgets.QPushButton('Connect')
        self._connBtn.clicked.connect(self.proc_btn_connect)

        self._uploadLocalLine = QtWidgets.QLineEdit()
        self._uploadLocalLine.setPlaceholderText('local path')
        self._uploadRemoteLine = QtWidgets.QLineEdit()
        self._uploadRemoteLine.setPlaceholderText('remote path')
        self._uploadBtn = QtWidgets.QPushButton('Upload')
        self._uploadBtn.setEnabled(False)
        self._uploadBtn.clicked.connect(self.proc_btn_upload)
        uploadLayout = QtWidgets.QHBoxLayout()
        uploadLayout.addWidget(self._uploadLocalLine)
        uploadLayout.addWidget(self._uploadRemoteLine)
        uploadLayout.addWidget(self._uploadBtn)

        self._downloadRemoteLine = QtWidgets.QLineEdit()
        self._downloadRemoteLine.setPlaceholderText('remote path')
        self._downloadLocalLine = QtWidgets.QLineEdit()
        self._downloadLocalLine.setPlaceholderText('local path')
        self._downloadBtn = QtWidgets.QPushButton('Download')
        self._downloadBtn.setEnabled(False)
        self._downloadBtn.clicked.connect(self.proc_btn_download)
        downloadLayout = QtWidgets.QHBoxLayout()
        downloadLayout.addWidget(self._downloadRemoteLine)
        downloadLayout.addWidget(self._downloadLocalLine)
        downloadLayout.addWidget(self._downloadBtn)

        self._inputBox = QtWidgets.QTextEdit()
        self._inputBox.setAcceptRichText(False)
        self._tabWidget = QtWidgets.QTabWidget()
        rightLayout = QtWidgets.QVBoxLayout()
        rightLayout.addWidget(self._connBtn)
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
                self._ssh = SSH.SSH('192.168.0.11', 'robocup', 'robocup')
                self._shell = self._ssh.create_shell()
                self._connected = True
                self._connBtn.setText('Disconnect')
                self._statusBar.showMessage('connected to {}@{}'.format('robocup', '192.168.0.11'))
                self._idSpin.setEnabled(False)
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
            self._idSpin.setEnabled(True)
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

    def proc_btn_upload(self):
        if self._ssh is None:
            return

    def proc_btn_download(self):
        if self._ssh is None:
            return
        
        

if __name__ == '__main__':
    if len(sys.argv) < 1:
        common.print_error('run with cfg file')
        exit(1)
    app = QtWidgets.QApplication(sys.argv)
    w = EasyWindow()
    w.show()
    sys.exit(app.exec_())