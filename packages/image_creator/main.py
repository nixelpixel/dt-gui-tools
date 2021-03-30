# -*- coding: utf-8 -*-
import sys

from PyQt5 import QtWidgets

from mainwindow import imageCreator


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = imageCreator()
    window.show()
    app.exec_()


if __name__ == '__main__':
    main()
