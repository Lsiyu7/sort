/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *pick_retreat_button;
    QPushButton *init_tool_button;
    QPushButton *pick_approach_button;
    QPushButton *cancel_tool_button;
    QTextEdit *textEdit_2;
    QLabel *label_2;
    QPushButton *stop_button;
    QPushButton *up_sleeve_button;
    QPushButton *tighten_bolt_button;
    QLabel *label_3;
    QLabel *label_4;
    QPushButton *rx_plus_button;
    QPushButton *rx_minus_button;
    QPushButton *ry_plus_button;
    QPushButton *rz_plus_button;
    QPushButton *ry_minus_button;
    QPushButton *rz_minus_button;
    QPushButton *x_plus_button;
    QPushButton *y_plus_button;
    QPushButton *y_minus_button;
    QPushButton *z_minus_button;
    QPushButton *z_plus_button;
    QPushButton *x_minus_button;
    QPushButton *pick_line_button_1;
    QPushButton *place_line_button_1;
    QPushButton *loost_bolt_button;
    QPushButton *down_sleeve_button;
    QPushButton *stop_tool_button;
    QPushButton *close_gripper_button;
    QPushButton *open_gripper_button;
    QPushButton *init_arm_button;
    QPushButton *stop_arm_button;
    QPushButton *close_arm_button;
    QPushButton *return_button;
    QPushButton *init_arm_controller_button;
    QPushButton *j1_plus_button;
    QPushButton *j1_minus_button;
    QPushButton *place_retreat_button;
    QPushButton *place_approach_button;
    QPushButton *j2_plus_button;
    QPushButton *j2_minus_button;
    QPushButton *j3_plus_button;
    QPushButton *j3_minus_button;
    QPushButton *j4_plus_button;
    QPushButton *j4_minus_button;
    QPushButton *j5_plus_button;
    QPushButton *j5_minus_button;
    QPushButton *j6_plus_button;
    QPushButton *j6_minus_button;
    QPushButton *pick_line_button_2;
    QPushButton *pick_line_button_3;
    QPushButton *place_line_button_2;
    QPushButton *place_line_button_3;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1344, 856);
        MainWindow->setMinimumSize(QSize(1040, 647));
        MainWindow->setMaximumSize(QSize(16777215, 16777215));
        QFont font;
        font.setPointSize(8);
        MainWindow->setFont(font);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        pick_retreat_button = new QPushButton(centralWidget);
        pick_retreat_button->setObjectName(QStringLiteral("pick_retreat_button"));
        pick_retreat_button->setEnabled(true);
        pick_retreat_button->setGeometry(QRect(1200, 140, 101, 81));
        QFont font1;
        font1.setPointSize(12);
        font1.setItalic(false);
        pick_retreat_button->setFont(font1);
        init_tool_button = new QPushButton(centralWidget);
        init_tool_button->setObjectName(QStringLiteral("init_tool_button"));
        init_tool_button->setGeometry(QRect(1110, 400, 81, 81));
        QFont font2;
        font2.setPointSize(12);
        init_tool_button->setFont(font2);
        pick_approach_button = new QPushButton(centralWidget);
        pick_approach_button->setObjectName(QStringLiteral("pick_approach_button"));
        pick_approach_button->setGeometry(QRect(1050, 140, 101, 81));
        pick_approach_button->setFont(font2);
        cancel_tool_button = new QPushButton(centralWidget);
        cancel_tool_button->setObjectName(QStringLiteral("cancel_tool_button"));
        cancel_tool_button->setEnabled(false);
        cancel_tool_button->setGeometry(QRect(1110, 510, 81, 81));
        cancel_tool_button->setFont(font2);
        textEdit_2 = new QTextEdit(centralWidget);
        textEdit_2->setObjectName(QStringLiteral("textEdit_2"));
        textEdit_2->setGeometry(QRect(530, 40, 351, 311));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(650, 0, 81, 31));
        label_2->setFont(font2);
        stop_button = new QPushButton(centralWidget);
        stop_button->setObjectName(QStringLiteral("stop_button"));
        stop_button->setGeometry(QRect(670, 400, 111, 191));
        QFont font3;
        font3.setPointSize(14);
        stop_button->setFont(font3);
        up_sleeve_button = new QPushButton(centralWidget);
        up_sleeve_button->setObjectName(QStringLiteral("up_sleeve_button"));
        up_sleeve_button->setGeometry(QRect(900, 400, 81, 81));
        up_sleeve_button->setFont(font2);
        tighten_bolt_button = new QPushButton(centralWidget);
        tighten_bolt_button->setObjectName(QStringLiteral("tighten_bolt_button"));
        tighten_bolt_button->setGeometry(QRect(800, 400, 81, 81));
        tighten_bolt_button->setFont(font2);
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(100, 0, 81, 20));
        label_3->setFont(font2);
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(340, 0, 81, 31));
        label_4->setFont(font2);
        rx_plus_button = new QPushButton(centralWidget);
        rx_plus_button->setObjectName(QStringLiteral("rx_plus_button"));
        rx_plus_button->setGeometry(QRect(20, 40, 101, 91));
        rx_plus_button->setFont(font2);
        rx_minus_button = new QPushButton(centralWidget);
        rx_minus_button->setObjectName(QStringLiteral("rx_minus_button"));
        rx_minus_button->setGeometry(QRect(150, 40, 101, 91));
        rx_minus_button->setFont(font2);
        ry_plus_button = new QPushButton(centralWidget);
        ry_plus_button->setObjectName(QStringLiteral("ry_plus_button"));
        ry_plus_button->setGeometry(QRect(20, 150, 101, 91));
        ry_plus_button->setFont(font2);
        rz_plus_button = new QPushButton(centralWidget);
        rz_plus_button->setObjectName(QStringLiteral("rz_plus_button"));
        rz_plus_button->setGeometry(QRect(20, 260, 101, 91));
        rz_plus_button->setFont(font2);
        ry_minus_button = new QPushButton(centralWidget);
        ry_minus_button->setObjectName(QStringLiteral("ry_minus_button"));
        ry_minus_button->setGeometry(QRect(150, 150, 101, 91));
        ry_minus_button->setFont(font2);
        rz_minus_button = new QPushButton(centralWidget);
        rz_minus_button->setObjectName(QStringLiteral("rz_minus_button"));
        rz_minus_button->setGeometry(QRect(150, 260, 101, 91));
        rz_minus_button->setFont(font2);
        x_plus_button = new QPushButton(centralWidget);
        x_plus_button->setObjectName(QStringLiteral("x_plus_button"));
        x_plus_button->setGeometry(QRect(280, 40, 101, 91));
        x_plus_button->setFont(font2);
        y_plus_button = new QPushButton(centralWidget);
        y_plus_button->setObjectName(QStringLiteral("y_plus_button"));
        y_plus_button->setGeometry(QRect(280, 150, 101, 91));
        y_plus_button->setFont(font2);
        y_minus_button = new QPushButton(centralWidget);
        y_minus_button->setObjectName(QStringLiteral("y_minus_button"));
        y_minus_button->setGeometry(QRect(400, 150, 101, 91));
        y_minus_button->setFont(font2);
        z_minus_button = new QPushButton(centralWidget);
        z_minus_button->setObjectName(QStringLiteral("z_minus_button"));
        z_minus_button->setGeometry(QRect(400, 260, 101, 91));
        z_minus_button->setFont(font2);
        z_plus_button = new QPushButton(centralWidget);
        z_plus_button->setObjectName(QStringLiteral("z_plus_button"));
        z_plus_button->setGeometry(QRect(280, 260, 101, 91));
        z_plus_button->setFont(font2);
        x_minus_button = new QPushButton(centralWidget);
        x_minus_button->setObjectName(QStringLiteral("x_minus_button"));
        x_minus_button->setGeometry(QRect(400, 40, 101, 91));
        x_minus_button->setFont(font2);
        pick_line_button_1 = new QPushButton(centralWidget);
        pick_line_button_1->setObjectName(QStringLiteral("pick_line_button_1"));
        pick_line_button_1->setGeometry(QRect(1050, 20, 101, 81));
        pick_line_button_1->setFont(font2);
        place_line_button_1 = new QPushButton(centralWidget);
        place_line_button_1->setObjectName(QStringLiteral("place_line_button_1"));
        place_line_button_1->setGeometry(QRect(1200, 20, 101, 81));
        place_line_button_1->setFont(font2);
        loost_bolt_button = new QPushButton(centralWidget);
        loost_bolt_button->setObjectName(QStringLiteral("loost_bolt_button"));
        loost_bolt_button->setGeometry(QRect(800, 510, 81, 81));
        loost_bolt_button->setFont(font2);
        down_sleeve_button = new QPushButton(centralWidget);
        down_sleeve_button->setObjectName(QStringLiteral("down_sleeve_button"));
        down_sleeve_button->setGeometry(QRect(900, 510, 81, 81));
        down_sleeve_button->setFont(font2);
        stop_tool_button = new QPushButton(centralWidget);
        stop_tool_button->setObjectName(QStringLiteral("stop_tool_button"));
        stop_tool_button->setGeometry(QRect(1210, 400, 101, 191));
        stop_tool_button->setFont(font3);
        close_gripper_button = new QPushButton(centralWidget);
        close_gripper_button->setObjectName(QStringLiteral("close_gripper_button"));
        close_gripper_button->setGeometry(QRect(1000, 400, 81, 81));
        close_gripper_button->setFont(font2);
        open_gripper_button = new QPushButton(centralWidget);
        open_gripper_button->setObjectName(QStringLiteral("open_gripper_button"));
        open_gripper_button->setGeometry(QRect(1000, 510, 81, 81));
        open_gripper_button->setFont(font2);
        init_arm_button = new QPushButton(centralWidget);
        init_arm_button->setObjectName(QStringLiteral("init_arm_button"));
        init_arm_button->setGeometry(QRect(940, 120, 81, 71));
        init_arm_button->setFont(font2);
        stop_arm_button = new QPushButton(centralWidget);
        stop_arm_button->setObjectName(QStringLiteral("stop_arm_button"));
        stop_arm_button->setGeometry(QRect(940, 200, 81, 71));
        stop_arm_button->setFont(font2);
        close_arm_button = new QPushButton(centralWidget);
        close_arm_button->setObjectName(QStringLiteral("close_arm_button"));
        close_arm_button->setEnabled(false);
        close_arm_button->setGeometry(QRect(940, 280, 81, 71));
        close_arm_button->setFont(font2);
        return_button = new QPushButton(centralWidget);
        return_button->setObjectName(QStringLiteral("return_button"));
        return_button->setGeometry(QRect(670, 610, 111, 191));
        return_button->setFont(font3);
        init_arm_controller_button = new QPushButton(centralWidget);
        init_arm_controller_button->setObjectName(QStringLiteral("init_arm_controller_button"));
        init_arm_controller_button->setGeometry(QRect(940, 20, 81, 91));
        init_arm_controller_button->setFont(font2);
        j1_plus_button = new QPushButton(centralWidget);
        j1_plus_button->setObjectName(QStringLiteral("j1_plus_button"));
        j1_plus_button->setGeometry(QRect(10, 400, 101, 191));
        QFont font4;
        font4.setPointSize(16);
        j1_plus_button->setFont(font4);
        j1_minus_button = new QPushButton(centralWidget);
        j1_minus_button->setObjectName(QStringLiteral("j1_minus_button"));
        j1_minus_button->setGeometry(QRect(120, 400, 101, 191));
        j1_minus_button->setFont(font4);
        place_retreat_button = new QPushButton(centralWidget);
        place_retreat_button->setObjectName(QStringLiteral("place_retreat_button"));
        place_retreat_button->setGeometry(QRect(1200, 270, 101, 81));
        place_retreat_button->setFont(font2);
        place_approach_button = new QPushButton(centralWidget);
        place_approach_button->setObjectName(QStringLiteral("place_approach_button"));
        place_approach_button->setGeometry(QRect(1050, 270, 101, 81));
        place_approach_button->setFont(font2);
        j2_plus_button = new QPushButton(centralWidget);
        j2_plus_button->setObjectName(QStringLiteral("j2_plus_button"));
        j2_plus_button->setGeometry(QRect(10, 610, 101, 191));
        j2_plus_button->setFont(font4);
        j2_minus_button = new QPushButton(centralWidget);
        j2_minus_button->setObjectName(QStringLiteral("j2_minus_button"));
        j2_minus_button->setGeometry(QRect(120, 610, 101, 191));
        j2_minus_button->setFont(font4);
        j3_plus_button = new QPushButton(centralWidget);
        j3_plus_button->setObjectName(QStringLiteral("j3_plus_button"));
        j3_plus_button->setGeometry(QRect(230, 400, 101, 191));
        j3_plus_button->setFont(font4);
        j3_minus_button = new QPushButton(centralWidget);
        j3_minus_button->setObjectName(QStringLiteral("j3_minus_button"));
        j3_minus_button->setGeometry(QRect(340, 400, 101, 191));
        j3_minus_button->setFont(font4);
        j4_plus_button = new QPushButton(centralWidget);
        j4_plus_button->setObjectName(QStringLiteral("j4_plus_button"));
        j4_plus_button->setGeometry(QRect(230, 610, 101, 191));
        j4_plus_button->setFont(font4);
        j4_minus_button = new QPushButton(centralWidget);
        j4_minus_button->setObjectName(QStringLiteral("j4_minus_button"));
        j4_minus_button->setGeometry(QRect(340, 610, 101, 191));
        j4_minus_button->setFont(font4);
        j5_plus_button = new QPushButton(centralWidget);
        j5_plus_button->setObjectName(QStringLiteral("j5_plus_button"));
        j5_plus_button->setGeometry(QRect(450, 400, 101, 191));
        j5_plus_button->setFont(font4);
        j5_minus_button = new QPushButton(centralWidget);
        j5_minus_button->setObjectName(QStringLiteral("j5_minus_button"));
        j5_minus_button->setGeometry(QRect(450, 610, 101, 191));
        j5_minus_button->setFont(font4);
        j6_plus_button = new QPushButton(centralWidget);
        j6_plus_button->setObjectName(QStringLiteral("j6_plus_button"));
        j6_plus_button->setGeometry(QRect(560, 400, 101, 191));
        j6_plus_button->setFont(font4);
        j6_minus_button = new QPushButton(centralWidget);
        j6_minus_button->setObjectName(QStringLiteral("j6_minus_button"));
        j6_minus_button->setGeometry(QRect(560, 610, 101, 191));
        j6_minus_button->setFont(font4);
        pick_line_button_2 = new QPushButton(centralWidget);
        pick_line_button_2->setObjectName(QStringLiteral("pick_line_button_2"));
        pick_line_button_2->setGeometry(QRect(810, 620, 101, 81));
        pick_line_button_2->setFont(font2);
        pick_line_button_3 = new QPushButton(centralWidget);
        pick_line_button_3->setObjectName(QStringLiteral("pick_line_button_3"));
        pick_line_button_3->setGeometry(QRect(930, 620, 101, 81));
        pick_line_button_3->setFont(font2);
        place_line_button_2 = new QPushButton(centralWidget);
        place_line_button_2->setObjectName(QStringLiteral("place_line_button_2"));
        place_line_button_2->setGeometry(QRect(810, 720, 101, 81));
        place_line_button_2->setFont(font2);
        place_line_button_3 = new QPushButton(centralWidget);
        place_line_button_3->setObjectName(QStringLiteral("place_line_button_3"));
        place_line_button_3->setGeometry(QRect(930, 720, 101, 81));
        place_line_button_3->setFont(font2);
        MainWindow->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "\345\210\206\346\213\243\346\234\272\345\231\250\344\272\272", 0));
        pick_retreat_button->setText(QApplication::translate("MainWindow", "\346\214\202-\346\222\244\345\233\236", 0));
        init_tool_button->setText(QApplication::translate("MainWindow", "\345\210\235\345\247\213\345\214\226\n"
"\345\267\245\345\205\267\347\253\257", 0));
        pick_approach_button->setText(QApplication::translate("MainWindow", "\346\214\202-\346\216\245\350\277\221", 0));
        cancel_tool_button->setText(QApplication::translate("MainWindow", "\346\226\255\345\274\200\n"
"\345\267\245\345\205\267\347\253\257", 0));
        label_2->setText(QApplication::translate("MainWindow", "\347\263\273\347\273\237\344\277\241\346\201\257", 0));
        stop_button->setText(QApplication::translate("MainWindow", "\346\234\272\346\242\260\350\207\202\n"
"\345\201\234\346\255\242\350\277\220\345\212\250", 0));
        up_sleeve_button->setText(QApplication::translate("MainWindow", "\345\245\227\347\255\222\344\270\212", 0));
        tighten_bolt_button->setText(QApplication::translate("MainWindow", "\347\264\247\350\236\272\346\240\223", 0));
        label_3->setText(QApplication::translate("MainWindow", "\345\247\277\346\200\201\346\216\247\345\210\266", 0));
        label_4->setText(QApplication::translate("MainWindow", "\344\275\215\347\275\256\346\216\247\345\210\266", 0));
        rx_plus_button->setText(QApplication::translate("MainWindow", "X+", 0));
        rx_minus_button->setText(QApplication::translate("MainWindow", "X-", 0));
        ry_plus_button->setText(QApplication::translate("MainWindow", "Y+", 0));
        rz_plus_button->setText(QApplication::translate("MainWindow", "Z+", 0));
        ry_minus_button->setText(QApplication::translate("MainWindow", "Y-", 0));
        rz_minus_button->setText(QApplication::translate("MainWindow", "Z-", 0));
        x_plus_button->setText(QApplication::translate("MainWindow", "X+", 0));
        y_plus_button->setText(QApplication::translate("MainWindow", "Y+", 0));
        y_minus_button->setText(QApplication::translate("MainWindow", "Y-", 0));
        z_minus_button->setText(QApplication::translate("MainWindow", "Z-", 0));
        z_plus_button->setText(QApplication::translate("MainWindow", "Z+", 0));
        x_minus_button->setText(QApplication::translate("MainWindow", "X-", 0));
        pick_line_button_1->setText(QApplication::translate("MainWindow", "\346\212\223\345\217\226\n"
"\346\216\245\345\234\260\347\272\277", 0));
        place_line_button_1->setText(QApplication::translate("MainWindow", "\346\224\276\345\233\236\n"
"\346\216\245\345\234\260\347\272\277", 0));
        loost_bolt_button->setText(QApplication::translate("MainWindow", "\346\235\276\350\236\272\346\240\223", 0));
        down_sleeve_button->setText(QApplication::translate("MainWindow", "\345\245\227\347\255\222\344\270\213", 0));
        stop_tool_button->setText(QApplication::translate("MainWindow", "\345\267\245\345\205\267\347\253\257\n"
"\345\201\234\346\255\242\350\277\220\350\241\214", 0));
        close_gripper_button->setText(QApplication::translate("MainWindow", "\347\264\247\345\244\271\347\210\252", 0));
        open_gripper_button->setText(QApplication::translate("MainWindow", "\346\235\276\345\244\271\347\210\252", 0));
        init_arm_button->setText(QApplication::translate("MainWindow", "\345\210\235\345\247\213\345\214\226\n"
"\346\234\272\346\242\260\350\207\202", 0));
        stop_arm_button->setText(QApplication::translate("MainWindow", "\346\234\272\346\242\260\350\207\202\n"
"\346\200\245\345\201\234", 0));
        close_arm_button->setText(QApplication::translate("MainWindow", "\345\205\263\351\227\255\n"
"\346\234\272\346\242\260\350\207\202", 0));
        return_button->setText(QApplication::translate("MainWindow", "\346\234\272\346\242\260\350\207\202\n"
"\345\233\236\345\210\235\345\247\213\344\275\215", 0));
        init_arm_controller_button->setText(QApplication::translate("MainWindow", "\346\234\272\346\242\260\350\207\202\n"
"\346\216\247\345\210\266\346\237\234\n"
"\344\270\212\347\224\265", 0));
        j1_plus_button->setText(QApplication::translate("MainWindow", "\345\205\263\350\212\2021\n"
"\350\275\264\345\212\250+", 0));
        j1_minus_button->setText(QApplication::translate("MainWindow", "\345\205\263\350\212\2021\n"
"\350\275\264\345\212\250-", 0));
        place_retreat_button->setText(QApplication::translate("MainWindow", "\346\221\230-\346\222\244\345\233\236", 0));
        place_approach_button->setText(QApplication::translate("MainWindow", "\346\221\230-\346\216\245\350\277\221", 0));
        j2_plus_button->setText(QApplication::translate("MainWindow", "\345\205\263\350\212\2022\n"
"\350\275\264\345\212\250+", 0));
        j2_minus_button->setText(QApplication::translate("MainWindow", "\345\205\263\350\212\2022\n"
"\350\275\264\345\212\250-", 0));
        j3_plus_button->setText(QApplication::translate("MainWindow", "\345\205\263\350\212\2023\n"
"\350\275\264\345\212\250+", 0));
        j3_minus_button->setText(QApplication::translate("MainWindow", "\345\205\263\350\212\2023\n"
"\350\275\264\345\212\250-", 0));
        j4_plus_button->setText(QApplication::translate("MainWindow", "\345\205\263\350\212\2024\n"
"\350\275\264\345\212\250+", 0));
        j4_minus_button->setText(QApplication::translate("MainWindow", "\345\205\263\350\212\2024\n"
"\350\275\264\345\212\250-", 0));
        j5_plus_button->setText(QApplication::translate("MainWindow", "\345\205\263\350\212\2025\n"
"\350\275\264\345\212\250+", 0));
        j5_minus_button->setText(QApplication::translate("MainWindow", "\345\205\263\350\212\2025\n"
"\350\275\264\345\212\250-", 0));
        j6_plus_button->setText(QApplication::translate("MainWindow", "\345\205\263\350\212\2026\n"
"\350\275\264\345\212\250+", 0));
        j6_minus_button->setText(QApplication::translate("MainWindow", "\345\205\263\350\212\2026\n"
"\350\275\264\345\212\250-", 0));
        pick_line_button_2->setText(QApplication::translate("MainWindow", "\346\212\223\345\217\226\n"
"\346\216\245\345\234\260\347\272\2772", 0));
        pick_line_button_3->setText(QApplication::translate("MainWindow", "\346\212\223\345\217\226\n"
"\346\216\245\345\234\2603", 0));
        place_line_button_2->setText(QApplication::translate("MainWindow", "\346\224\276\345\233\236\n"
"\346\216\245\345\234\260\347\272\2772", 0));
        place_line_button_3->setText(QApplication::translate("MainWindow", "\346\224\276\345\233\236\n"
"\346\216\245\345\234\260\347\272\2773", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
