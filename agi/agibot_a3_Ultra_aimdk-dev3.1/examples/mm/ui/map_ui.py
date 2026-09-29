# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'map.ui'
##
## Created by: Qt User Interface Compiler version 6.10.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QDialog, QGraphicsView, QLabel,
    QLineEdit, QListWidget, QListWidgetItem, QPushButton,
    QSizePolicy, QWidget)

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(1000, 690)
        self.create_map_button = QPushButton(Dialog)
        self.create_map_button.setObjectName(u"create_map_button")
        self.create_map_button.setGeometry(QRect(20, 640, 91, 41))
        self.canncel_map_button = QPushButton(Dialog)
        self.canncel_map_button.setObjectName(u"canncel_map_button")
        self.canncel_map_button.setGeometry(QRect(240, 640, 91, 41))
        self.save_map_button = QPushButton(Dialog)
        self.save_map_button.setObjectName(u"save_map_button")
        self.save_map_button.setGeometry(QRect(130, 640, 91, 41))
        self.ip_input = QLineEdit(Dialog)
        self.ip_input.setObjectName(u"ip_input")
        self.ip_input.setGeometry(QRect(70, 590, 113, 25))
        self.label = QLabel(Dialog)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 590, 81, 21))
        self.graphicsView = QGraphicsView(Dialog)
        self.graphicsView.setObjectName(u"graphicsView")
        self.graphicsView.setGeometry(QRect(30, 10, 721, 561))
        self.connect_button = QPushButton(Dialog)
        self.connect_button.setObjectName(u"connect_button")
        self.connect_button.setGeometry(QRect(200, 580, 91, 41))
        self.show_some = QLabel(Dialog)
        self.show_some.setObjectName(u"show_some")
        self.show_some.setGeometry(QRect(320, 590, 221, 17))
        self.draw_button = QPushButton(Dialog)
        self.draw_button.setObjectName(u"draw_button")
        self.draw_button.setGeometry(QRect(550, 580, 91, 41))
        self.unknow_button = QPushButton(Dialog)
        self.unknow_button.setObjectName(u"unknow_button")
        self.unknow_button.setGeometry(QRect(550, 640, 91, 41))
        self.map_list = QListWidget(Dialog)
        self.map_list.setObjectName(u"map_list")
        self.map_list.setGeometry(QRect(760, 30, 211, 541))
        self.label_2 = QLabel(Dialog)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(830, 10, 61, 21))
        self.select_map_button = QPushButton(Dialog)
        self.select_map_button.setObjectName(u"select_map_button")
        self.select_map_button.setGeometry(QRect(760, 580, 91, 41))
        self.delete_map_button = QPushButton(Dialog)
        self.delete_map_button.setObjectName(u"delete_map_button")
        self.delete_map_button.setGeometry(QRect(760, 640, 91, 41))
        self.relocation_button = QPushButton(Dialog)
        self.relocation_button.setObjectName(u"relocation_button")
        self.relocation_button.setGeometry(QRect(870, 580, 91, 41))
        self.get_map_list_button = QPushButton(Dialog)
        self.get_map_list_button.setObjectName(u"get_map_list_button")
        self.get_map_list_button.setGeometry(QRect(870, 640, 89, 41))
        self.clear_button = QPushButton(Dialog)
        self.clear_button.setObjectName(u"clear_button")
        self.clear_button.setGeometry(QRect(650, 580, 91, 41))
        self.rename_map_button = QPushButton(Dialog)
        self.rename_map_button.setObjectName(u"rename_map_button")
        self.rename_map_button.setGeometry(QRect(650, 640, 91, 41))
        self.show_current_working_map = QLabel(Dialog)
        self.show_current_working_map.setObjectName(u"show_current_working_map")
        self.show_current_working_map.setGeometry(QRect(290, 20, 251, 17))

        self.retranslateUi(Dialog)

        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"Dialog", None))
        self.create_map_button.setText(QCoreApplication.translate("Dialog", u"\u521b\u5efa\u5730\u56fe", None))
        self.canncel_map_button.setText(QCoreApplication.translate("Dialog", u"\u53d6\u6d88\u5efa\u56fe", None))
        self.save_map_button.setText(QCoreApplication.translate("Dialog", u"\u4fdd\u5b58\u5efa\u56fe", None))
        self.label.setText(QCoreApplication.translate("Dialog", u"IP\u8f93\u5165\uff1a", None))
        self.connect_button.setText(QCoreApplication.translate("Dialog", u"\u6d4b\u8bd5\u8fde\u63a5", None))
        self.show_some.setText(QCoreApplication.translate("Dialog", u"\u4fe1\u606f\u6846", None))
        self.draw_button.setText(QCoreApplication.translate("Dialog", u"\u7ed8\u5236\u6a21\u5f0f", None))
        self.unknow_button.setText(QCoreApplication.translate("Dialog", u"\u672a\u77e5\u533a\u57df", None))
        self.label_2.setText(QCoreApplication.translate("Dialog", u"\u5730\u56fe\u5217\u8868", None))
        self.select_map_button.setText(QCoreApplication.translate("Dialog", u"\u5207\u6362\u5730\u56fe", None))
        self.delete_map_button.setText(QCoreApplication.translate("Dialog", u"\u5220\u9664\u5730\u56fe", None))
        self.relocation_button.setText(QCoreApplication.translate("Dialog", u"\u91cd\u5b9a\u4f4d", None))
        self.get_map_list_button.setText(QCoreApplication.translate("Dialog", u"\u5237\u65b0\u5217\u8868", None))
        self.clear_button.setText(QCoreApplication.translate("Dialog", u"\u6e05\u7a7a\u7ed8\u753b", None))
        self.rename_map_button.setText(QCoreApplication.translate("Dialog", u"\u91cd\u547d\u540d\u5730\u56fe", None))
        self.show_current_working_map.setText("")
    # retranslateUi

