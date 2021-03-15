# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main_window.ui'
#
# Created by: PyQt5 UI code generator 5.12.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1945, 959)
        MainWindow.setWindowOpacity(1.0)
        MainWindow.setDockOptions(QtWidgets.QMainWindow.AllowTabbedDocks|QtWidgets.QMainWindow.AnimatedDocks)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_11 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_11.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_11.setObjectName("gridLayout_11")
        self.container_horizontal_layout = QtWidgets.QHBoxLayout()
        self.container_horizontal_layout.setObjectName("container_horizontal_layout")
        self.gridLayout_11.addLayout(self.container_horizontal_layout, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1945, 31))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuWindow = QtWidgets.QMenu(self.menubar)
        self.menuWindow.setObjectName("menuWindow")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.regularization_toolbar = QtWidgets.QToolBar(MainWindow)
        self.regularization_toolbar.setObjectName("regularization_toolbar")
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.regularization_toolbar)
        self.actionSiderBar = QtWidgets.QAction(MainWindow)
        self.actionSiderBar.setCheckable(True)
        self.actionSiderBar.setChecked(True)
        self.actionSiderBar.setObjectName("actionSiderBar")
        self.actionHideAll = QtWidgets.QAction(MainWindow)
        self.actionHideAll.setObjectName("actionHideAll")
        self.actionShowAll = QtWidgets.QAction(MainWindow)
        self.actionShowAll.setObjectName("actionShowAll")
        self.actionLoadCorrespondences = QtWidgets.QAction(MainWindow)
        self.actionLoadCorrespondences.setObjectName("actionLoadCorrespondences")
        self.actionLoadVectorMap = QtWidgets.QAction(MainWindow)
        self.actionLoadVectorMap.setObjectName("actionLoadVectorMap")
        self.actionLoadImages = QtWidgets.QAction(MainWindow)
        self.actionLoadImages.setObjectName("actionLoadImages")
        self.actionSaveCorrespondences = QtWidgets.QAction(MainWindow)
        self.actionSaveCorrespondences.setObjectName("actionSaveCorrespondences")
        self.actionOptimizeCalibration = QtWidgets.QAction(MainWindow)
        self.actionOptimizeCalibration.setObjectName("actionOptimizeCalibration")
        self.menuFile.addAction(self.actionLoadCorrespondences)
        self.menuFile.addSeparator()
        self.menuFile.addSeparator()
        self.menuFile.addAction(self.actionLoadImages)
        self.menuFile.addAction(self.actionLoadVectorMap)
        self.menuFile.addSeparator()
        self.menuFile.addSeparator()
        self.menuWindow.addAction(self.actionSiderBar)
        self.menuWindow.addSeparator()
        self.menuWindow.addSeparator()
        self.menuWindow.addAction(self.actionHideAll)
        self.menuWindow.addAction(self.actionShowAll)
        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuWindow.menuAction())
        self.regularization_toolbar.addAction(self.actionLoadImages)
        self.regularization_toolbar.addAction(self.actionLoadVectorMap)
        self.regularization_toolbar.addAction(self.actionLoadCorrespondences)
        self.regularization_toolbar.addAction(self.actionSaveCorrespondences)
        self.regularization_toolbar.addAction(self.actionOptimizeCalibration)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MapBasedCalibrationTool"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))
        self.menuWindow.setTitle(_translate("MainWindow", "Window"))
        self.regularization_toolbar.setWindowTitle(_translate("MainWindow", "toolBar"))
        self.actionSiderBar.setText(_translate("MainWindow", "Sider Bar"))
        self.actionSiderBar.setToolTip(_translate("MainWindow", "Sider Bar"))
        self.actionHideAll.setText(_translate("MainWindow", "Hide All"))
        self.actionShowAll.setText(_translate("MainWindow", "Show All"))
        self.actionLoadCorrespondences.setText(_translate("MainWindow", "Load Correspondences"))
        self.actionLoadCorrespondences.setToolTip(_translate("MainWindow", "Load Correspondences"))
        self.actionLoadVectorMap.setText(_translate("MainWindow", "Load Vector Map"))
        self.actionLoadImages.setText(_translate("MainWindow", "Load Images"))
        self.actionSaveCorrespondences.setText(_translate("MainWindow", "Save Correspondences"))
        self.actionOptimizeCalibration.setText(_translate("MainWindow", "Optimize Calibration"))


from . import resources
