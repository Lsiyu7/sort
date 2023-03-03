/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[81];
    char stringdata0[2106];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 9), // "fail_info"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 4), // "info"
QT_MOC_LITERAL(4, 27, 8), // "tool_cmd"
QT_MOC_LITERAL(5, 36, 3), // "cmd"
QT_MOC_LITERAL(6, 40, 22), // "on_stop_button_clicked"
QT_MOC_LITERAL(7, 63, 30), // "on_open_gripper_button_clicked"
QT_MOC_LITERAL(8, 94, 31), // "on_pick_approach_button_clicked"
QT_MOC_LITERAL(9, 126, 30), // "on_pick_retreat_button_clicked"
QT_MOC_LITERAL(10, 157, 31), // "on_close_gripper_button_clicked"
QT_MOC_LITERAL(11, 189, 16), // "socket_Read_Data"
QT_MOC_LITERAL(12, 206, 18), // "server_New_Connect"
QT_MOC_LITERAL(13, 225, 27), // "on_init_tool_button_clicked"
QT_MOC_LITERAL(14, 253, 29), // "on_cancel_tool_button_clicked"
QT_MOC_LITERAL(15, 283, 27), // "on_stop_tool_button_clicked"
QT_MOC_LITERAL(16, 311, 27), // "on_up_sleeve_button_clicked"
QT_MOC_LITERAL(17, 339, 29), // "on_down_sleeve_button_clicked"
QT_MOC_LITERAL(18, 369, 30), // "on_tighten_bolt_button_clicked"
QT_MOC_LITERAL(19, 400, 28), // "on_loost_bolt_button_clicked"
QT_MOC_LITERAL(20, 429, 26), // "on_init_arm_button_clicked"
QT_MOC_LITERAL(21, 456, 26), // "on_stop_arm_button_clicked"
QT_MOC_LITERAL(22, 483, 27), // "on_close_arm_button_clicked"
QT_MOC_LITERAL(23, 511, 24), // "on_return_button_clicked"
QT_MOC_LITERAL(24, 536, 37), // "on_init_arm_controller_button..."
QT_MOC_LITERAL(25, 574, 32), // "on_place_approach_button_clicked"
QT_MOC_LITERAL(26, 607, 31), // "on_place_retreat_button_clicked"
QT_MOC_LITERAL(27, 639, 24), // "on_x_plus_button_pressed"
QT_MOC_LITERAL(28, 664, 25), // "on_x_plus_button_released"
QT_MOC_LITERAL(29, 690, 25), // "on_x_minus_button_pressed"
QT_MOC_LITERAL(30, 716, 26), // "on_x_minus_button_released"
QT_MOC_LITERAL(31, 743, 24), // "on_y_plus_button_pressed"
QT_MOC_LITERAL(32, 768, 25), // "on_y_plus_button_released"
QT_MOC_LITERAL(33, 794, 25), // "on_y_minus_button_pressed"
QT_MOC_LITERAL(34, 820, 26), // "on_y_minus_button_released"
QT_MOC_LITERAL(35, 847, 24), // "on_z_plus_button_pressed"
QT_MOC_LITERAL(36, 872, 25), // "on_z_plus_button_released"
QT_MOC_LITERAL(37, 898, 25), // "on_z_minus_button_pressed"
QT_MOC_LITERAL(38, 924, 26), // "on_z_minus_button_released"
QT_MOC_LITERAL(39, 951, 25), // "on_rx_plus_button_pressed"
QT_MOC_LITERAL(40, 977, 26), // "on_rx_plus_button_released"
QT_MOC_LITERAL(41, 1004, 26), // "on_rx_minus_button_pressed"
QT_MOC_LITERAL(42, 1031, 27), // "on_rx_minus_button_released"
QT_MOC_LITERAL(43, 1059, 25), // "on_ry_plus_button_pressed"
QT_MOC_LITERAL(44, 1085, 26), // "on_ry_plus_button_released"
QT_MOC_LITERAL(45, 1112, 26), // "on_ry_minus_button_pressed"
QT_MOC_LITERAL(46, 1139, 27), // "on_ry_minus_button_released"
QT_MOC_LITERAL(47, 1167, 25), // "on_rz_plus_button_pressed"
QT_MOC_LITERAL(48, 1193, 26), // "on_rz_plus_button_released"
QT_MOC_LITERAL(49, 1220, 26), // "on_rz_minus_button_pressed"
QT_MOC_LITERAL(50, 1247, 27), // "on_rz_minus_button_released"
QT_MOC_LITERAL(51, 1275, 25), // "on_j1_plus_button_pressed"
QT_MOC_LITERAL(52, 1301, 26), // "on_j1_plus_button_released"
QT_MOC_LITERAL(53, 1328, 26), // "on_j1_minus_button_pressed"
QT_MOC_LITERAL(54, 1355, 27), // "on_j1_minus_button_released"
QT_MOC_LITERAL(55, 1383, 25), // "on_j2_plus_button_pressed"
QT_MOC_LITERAL(56, 1409, 26), // "on_j2_plus_button_released"
QT_MOC_LITERAL(57, 1436, 26), // "on_j2_minus_button_pressed"
QT_MOC_LITERAL(58, 1463, 27), // "on_j2_minus_button_released"
QT_MOC_LITERAL(59, 1491, 25), // "on_j3_plus_button_pressed"
QT_MOC_LITERAL(60, 1517, 26), // "on_j3_plus_button_released"
QT_MOC_LITERAL(61, 1544, 26), // "on_j3_minus_button_pressed"
QT_MOC_LITERAL(62, 1571, 27), // "on_j3_minus_button_released"
QT_MOC_LITERAL(63, 1599, 25), // "on_j4_plus_button_pressed"
QT_MOC_LITERAL(64, 1625, 26), // "on_j4_plus_button_released"
QT_MOC_LITERAL(65, 1652, 26), // "on_j4_minus_button_pressed"
QT_MOC_LITERAL(66, 1679, 27), // "on_j4_minus_button_released"
QT_MOC_LITERAL(67, 1707, 25), // "on_j5_plus_button_pressed"
QT_MOC_LITERAL(68, 1733, 26), // "on_j5_plus_button_released"
QT_MOC_LITERAL(69, 1760, 26), // "on_j5_minus_button_pressed"
QT_MOC_LITERAL(70, 1787, 27), // "on_j5_minus_button_released"
QT_MOC_LITERAL(71, 1815, 25), // "on_j6_plus_button_pressed"
QT_MOC_LITERAL(72, 1841, 26), // "on_j6_plus_button_released"
QT_MOC_LITERAL(73, 1868, 26), // "on_j6_minus_button_pressed"
QT_MOC_LITERAL(74, 1895, 27), // "on_j6_minus_button_released"
QT_MOC_LITERAL(75, 1923, 29), // "on_pick_line_button_2_clicked"
QT_MOC_LITERAL(76, 1953, 29), // "on_pick_line_button_3_clicked"
QT_MOC_LITERAL(77, 1983, 30), // "on_place_line_button_2_clicked"
QT_MOC_LITERAL(78, 2014, 30), // "on_place_line_button_3_clicked"
QT_MOC_LITERAL(79, 2045, 29), // "on_pick_line_button_1_clicked"
QT_MOC_LITERAL(80, 2075, 30) // "on_place_line_button_1_clicked"

    },
    "MainWindow\0fail_info\0\0info\0tool_cmd\0"
    "cmd\0on_stop_button_clicked\0"
    "on_open_gripper_button_clicked\0"
    "on_pick_approach_button_clicked\0"
    "on_pick_retreat_button_clicked\0"
    "on_close_gripper_button_clicked\0"
    "socket_Read_Data\0server_New_Connect\0"
    "on_init_tool_button_clicked\0"
    "on_cancel_tool_button_clicked\0"
    "on_stop_tool_button_clicked\0"
    "on_up_sleeve_button_clicked\0"
    "on_down_sleeve_button_clicked\0"
    "on_tighten_bolt_button_clicked\0"
    "on_loost_bolt_button_clicked\0"
    "on_init_arm_button_clicked\0"
    "on_stop_arm_button_clicked\0"
    "on_close_arm_button_clicked\0"
    "on_return_button_clicked\0"
    "on_init_arm_controller_button_clicked\0"
    "on_place_approach_button_clicked\0"
    "on_place_retreat_button_clicked\0"
    "on_x_plus_button_pressed\0"
    "on_x_plus_button_released\0"
    "on_x_minus_button_pressed\0"
    "on_x_minus_button_released\0"
    "on_y_plus_button_pressed\0"
    "on_y_plus_button_released\0"
    "on_y_minus_button_pressed\0"
    "on_y_minus_button_released\0"
    "on_z_plus_button_pressed\0"
    "on_z_plus_button_released\0"
    "on_z_minus_button_pressed\0"
    "on_z_minus_button_released\0"
    "on_rx_plus_button_pressed\0"
    "on_rx_plus_button_released\0"
    "on_rx_minus_button_pressed\0"
    "on_rx_minus_button_released\0"
    "on_ry_plus_button_pressed\0"
    "on_ry_plus_button_released\0"
    "on_ry_minus_button_pressed\0"
    "on_ry_minus_button_released\0"
    "on_rz_plus_button_pressed\0"
    "on_rz_plus_button_released\0"
    "on_rz_minus_button_pressed\0"
    "on_rz_minus_button_released\0"
    "on_j1_plus_button_pressed\0"
    "on_j1_plus_button_released\0"
    "on_j1_minus_button_pressed\0"
    "on_j1_minus_button_released\0"
    "on_j2_plus_button_pressed\0"
    "on_j2_plus_button_released\0"
    "on_j2_minus_button_pressed\0"
    "on_j2_minus_button_released\0"
    "on_j3_plus_button_pressed\0"
    "on_j3_plus_button_released\0"
    "on_j3_minus_button_pressed\0"
    "on_j3_minus_button_released\0"
    "on_j4_plus_button_pressed\0"
    "on_j4_plus_button_released\0"
    "on_j4_minus_button_pressed\0"
    "on_j4_minus_button_released\0"
    "on_j5_plus_button_pressed\0"
    "on_j5_plus_button_released\0"
    "on_j5_minus_button_pressed\0"
    "on_j5_minus_button_released\0"
    "on_j6_plus_button_pressed\0"
    "on_j6_plus_button_released\0"
    "on_j6_minus_button_pressed\0"
    "on_j6_minus_button_released\0"
    "on_pick_line_button_2_clicked\0"
    "on_pick_line_button_3_clicked\0"
    "on_place_line_button_2_clicked\0"
    "on_place_line_button_3_clicked\0"
    "on_pick_line_button_1_clicked\0"
    "on_place_line_button_1_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      77,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,  399,    2, 0x08 /* Private */,
       4,    1,  402,    2, 0x08 /* Private */,
       6,    0,  405,    2, 0x08 /* Private */,
       7,    0,  406,    2, 0x08 /* Private */,
       8,    0,  407,    2, 0x08 /* Private */,
       9,    0,  408,    2, 0x08 /* Private */,
      10,    0,  409,    2, 0x08 /* Private */,
      11,    0,  410,    2, 0x08 /* Private */,
      12,    0,  411,    2, 0x08 /* Private */,
      13,    0,  412,    2, 0x08 /* Private */,
      14,    0,  413,    2, 0x08 /* Private */,
      15,    0,  414,    2, 0x08 /* Private */,
      16,    0,  415,    2, 0x08 /* Private */,
      17,    0,  416,    2, 0x08 /* Private */,
      18,    0,  417,    2, 0x08 /* Private */,
      19,    0,  418,    2, 0x08 /* Private */,
      20,    0,  419,    2, 0x08 /* Private */,
      21,    0,  420,    2, 0x08 /* Private */,
      22,    0,  421,    2, 0x08 /* Private */,
      23,    0,  422,    2, 0x08 /* Private */,
      24,    0,  423,    2, 0x08 /* Private */,
      25,    0,  424,    2, 0x08 /* Private */,
      26,    0,  425,    2, 0x08 /* Private */,
      27,    0,  426,    2, 0x08 /* Private */,
      28,    0,  427,    2, 0x08 /* Private */,
      29,    0,  428,    2, 0x08 /* Private */,
      30,    0,  429,    2, 0x08 /* Private */,
      31,    0,  430,    2, 0x08 /* Private */,
      32,    0,  431,    2, 0x08 /* Private */,
      33,    0,  432,    2, 0x08 /* Private */,
      34,    0,  433,    2, 0x08 /* Private */,
      35,    0,  434,    2, 0x08 /* Private */,
      36,    0,  435,    2, 0x08 /* Private */,
      37,    0,  436,    2, 0x08 /* Private */,
      38,    0,  437,    2, 0x08 /* Private */,
      39,    0,  438,    2, 0x08 /* Private */,
      40,    0,  439,    2, 0x08 /* Private */,
      41,    0,  440,    2, 0x08 /* Private */,
      42,    0,  441,    2, 0x08 /* Private */,
      43,    0,  442,    2, 0x08 /* Private */,
      44,    0,  443,    2, 0x08 /* Private */,
      45,    0,  444,    2, 0x08 /* Private */,
      46,    0,  445,    2, 0x08 /* Private */,
      47,    0,  446,    2, 0x08 /* Private */,
      48,    0,  447,    2, 0x08 /* Private */,
      49,    0,  448,    2, 0x08 /* Private */,
      50,    0,  449,    2, 0x08 /* Private */,
      51,    0,  450,    2, 0x08 /* Private */,
      52,    0,  451,    2, 0x08 /* Private */,
      53,    0,  452,    2, 0x08 /* Private */,
      54,    0,  453,    2, 0x08 /* Private */,
      55,    0,  454,    2, 0x08 /* Private */,
      56,    0,  455,    2, 0x08 /* Private */,
      57,    0,  456,    2, 0x08 /* Private */,
      58,    0,  457,    2, 0x08 /* Private */,
      59,    0,  458,    2, 0x08 /* Private */,
      60,    0,  459,    2, 0x08 /* Private */,
      61,    0,  460,    2, 0x08 /* Private */,
      62,    0,  461,    2, 0x08 /* Private */,
      63,    0,  462,    2, 0x08 /* Private */,
      64,    0,  463,    2, 0x08 /* Private */,
      65,    0,  464,    2, 0x08 /* Private */,
      66,    0,  465,    2, 0x08 /* Private */,
      67,    0,  466,    2, 0x08 /* Private */,
      68,    0,  467,    2, 0x08 /* Private */,
      69,    0,  468,    2, 0x08 /* Private */,
      70,    0,  469,    2, 0x08 /* Private */,
      71,    0,  470,    2, 0x08 /* Private */,
      72,    0,  471,    2, 0x08 /* Private */,
      73,    0,  472,    2, 0x08 /* Private */,
      74,    0,  473,    2, 0x08 /* Private */,
      75,    0,  474,    2, 0x08 /* Private */,
      76,    0,  475,    2, 0x08 /* Private */,
      77,    0,  476,    2, 0x08 /* Private */,
      78,    0,  477,    2, 0x08 /* Private */,
      79,    0,  478,    2, 0x08 /* Private */,
      80,    0,  479,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    5,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->fail_info((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->tool_cmd((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->on_stop_button_clicked(); break;
        case 3: _t->on_open_gripper_button_clicked(); break;
        case 4: _t->on_pick_approach_button_clicked(); break;
        case 5: _t->on_pick_retreat_button_clicked(); break;
        case 6: _t->on_close_gripper_button_clicked(); break;
        case 7: _t->socket_Read_Data(); break;
        case 8: _t->server_New_Connect(); break;
        case 9: _t->on_init_tool_button_clicked(); break;
        case 10: _t->on_cancel_tool_button_clicked(); break;
        case 11: _t->on_stop_tool_button_clicked(); break;
        case 12: _t->on_up_sleeve_button_clicked(); break;
        case 13: _t->on_down_sleeve_button_clicked(); break;
        case 14: _t->on_tighten_bolt_button_clicked(); break;
        case 15: _t->on_loost_bolt_button_clicked(); break;
        case 16: _t->on_init_arm_button_clicked(); break;
        case 17: _t->on_stop_arm_button_clicked(); break;
        case 18: _t->on_close_arm_button_clicked(); break;
        case 19: _t->on_return_button_clicked(); break;
        case 20: _t->on_init_arm_controller_button_clicked(); break;
        case 21: _t->on_place_approach_button_clicked(); break;
        case 22: _t->on_place_retreat_button_clicked(); break;
        case 23: _t->on_x_plus_button_pressed(); break;
        case 24: _t->on_x_plus_button_released(); break;
        case 25: _t->on_x_minus_button_pressed(); break;
        case 26: _t->on_x_minus_button_released(); break;
        case 27: _t->on_y_plus_button_pressed(); break;
        case 28: _t->on_y_plus_button_released(); break;
        case 29: _t->on_y_minus_button_pressed(); break;
        case 30: _t->on_y_minus_button_released(); break;
        case 31: _t->on_z_plus_button_pressed(); break;
        case 32: _t->on_z_plus_button_released(); break;
        case 33: _t->on_z_minus_button_pressed(); break;
        case 34: _t->on_z_minus_button_released(); break;
        case 35: _t->on_rx_plus_button_pressed(); break;
        case 36: _t->on_rx_plus_button_released(); break;
        case 37: _t->on_rx_minus_button_pressed(); break;
        case 38: _t->on_rx_minus_button_released(); break;
        case 39: _t->on_ry_plus_button_pressed(); break;
        case 40: _t->on_ry_plus_button_released(); break;
        case 41: _t->on_ry_minus_button_pressed(); break;
        case 42: _t->on_ry_minus_button_released(); break;
        case 43: _t->on_rz_plus_button_pressed(); break;
        case 44: _t->on_rz_plus_button_released(); break;
        case 45: _t->on_rz_minus_button_pressed(); break;
        case 46: _t->on_rz_minus_button_released(); break;
        case 47: _t->on_j1_plus_button_pressed(); break;
        case 48: _t->on_j1_plus_button_released(); break;
        case 49: _t->on_j1_minus_button_pressed(); break;
        case 50: _t->on_j1_minus_button_released(); break;
        case 51: _t->on_j2_plus_button_pressed(); break;
        case 52: _t->on_j2_plus_button_released(); break;
        case 53: _t->on_j2_minus_button_pressed(); break;
        case 54: _t->on_j2_minus_button_released(); break;
        case 55: _t->on_j3_plus_button_pressed(); break;
        case 56: _t->on_j3_plus_button_released(); break;
        case 57: _t->on_j3_minus_button_pressed(); break;
        case 58: _t->on_j3_minus_button_released(); break;
        case 59: _t->on_j4_plus_button_pressed(); break;
        case 60: _t->on_j4_plus_button_released(); break;
        case 61: _t->on_j4_minus_button_pressed(); break;
        case 62: _t->on_j4_minus_button_released(); break;
        case 63: _t->on_j5_plus_button_pressed(); break;
        case 64: _t->on_j5_plus_button_released(); break;
        case 65: _t->on_j5_minus_button_pressed(); break;
        case 66: _t->on_j5_minus_button_released(); break;
        case 67: _t->on_j6_plus_button_pressed(); break;
        case 68: _t->on_j6_plus_button_released(); break;
        case 69: _t->on_j6_minus_button_pressed(); break;
        case 70: _t->on_j6_minus_button_released(); break;
        case 71: _t->on_pick_line_button_2_clicked(); break;
        case 72: _t->on_pick_line_button_3_clicked(); break;
        case 73: _t->on_place_line_button_2_clicked(); break;
        case 74: _t->on_place_line_button_3_clicked(); break;
        case 75: _t->on_pick_line_button_1_clicked(); break;
        case 76: _t->on_place_line_button_1_clicked(); break;
        default: ;
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 77)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 77;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 77)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 77;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
