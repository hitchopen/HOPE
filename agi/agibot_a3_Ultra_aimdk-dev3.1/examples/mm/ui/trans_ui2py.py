from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QFile
import sys

from PySide6 import QtCore, QtWidgets, QtUiTools
import io

def convert_ui_to_py(ui_file, py_file):
    from PySide6 import QtCore
    from PySide6.QtUiTools import QUiLoader
    from PySide6.QtWidgets import QApplication
    import sys
    from PySide6 import QtUiTools
    from PySide6.QtUiTools import loadUiType
    import os
    os.system(f"pyside6-uic {ui_file} -o {py_file}")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 3:
        print("用法: python convert_ui.py map.ui map_ui.py")
        sys.exit(1)
    convert_ui_to_py(sys.argv[1], sys.argv[2])