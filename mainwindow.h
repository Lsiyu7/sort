#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QLineEdit>
#include <QPushButton>
#include <QTextEdit>
#include <QScrollBar>
#include <QtNetwork>
#include <QXmlStreamReader>
#include <QMap>
#include <QFile>
#include "orientation.h"
#include "thread_sort.h"
#include <QEventLoop>
#include "fjrobot.h"
#include "sample.h"

#include <QBuffer>
#include <QSharedMemory>
#include <QDataStream>
#include <QStringList>
#include <QString>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);

    void Delay(int msec);

    void get_image();

    void recognize(float r1,float c1,float r2,float c2);

    void on_up_sleeve();

    void on_down_sleeve();

    void on_tighten_bolt();

    void on_loost_bolt();

    void on_open_gripper();

    void on_close_gripper();

    ~MainWindow();

private slots:

    void fail_info(QString info);

    void tool_cmd(QString cmd);

    void on_stop_button_clicked();

    void on_open_gripper_button_clicked();

    void on_pick_approach_button_clicked();

    void on_pick_retreat_button_clicked();

    void on_close_gripper_button_clicked();

    void socket_Read_Data();

    void server_New_Connect();

    void on_init_tool_button_clicked();

    void on_cancel_tool_button_clicked();

    void on_stop_tool_button_clicked();

    void on_up_sleeve_button_clicked();

    void on_down_sleeve_button_clicked();

    void on_tighten_bolt_button_clicked();

    void on_loost_bolt_button_clicked();

    void on_init_arm_button_clicked();

    void on_stop_arm_button_clicked();

    void on_close_arm_button_clicked();

    void on_return_button_clicked();

    void on_init_arm_controller_button_clicked();

    void on_place_approach_button_clicked();

    void on_place_retreat_button_clicked();

    void on_x_plus_button_pressed();

    void on_x_plus_button_released();

    void on_x_minus_button_pressed();

    void on_x_minus_button_released();

    void on_y_plus_button_pressed();

    void on_y_plus_button_released();

    void on_y_minus_button_pressed();

    void on_y_minus_button_released();

    void on_z_plus_button_pressed();

    void on_z_plus_button_released();

    void on_z_minus_button_pressed();

    void on_z_minus_button_released();

    void on_rx_plus_button_pressed();

    void on_rx_plus_button_released();

    void on_rx_minus_button_pressed();

    void on_rx_minus_button_released();

    void on_ry_plus_button_pressed();

    void on_ry_plus_button_released();

    void on_ry_minus_button_pressed();

    void on_ry_minus_button_released();

    void on_rz_plus_button_pressed();

    void on_rz_plus_button_released();

    void on_rz_minus_button_pressed();

    void on_rz_minus_button_released();

    void on_j1_plus_button_pressed();

    void on_j1_plus_button_released();

    void on_j1_minus_button_pressed();

    void on_j1_minus_button_released();

    void on_j2_plus_button_pressed();

    void on_j2_plus_button_released();

    void on_j2_minus_button_pressed();

    void on_j2_minus_button_released();

    void on_j3_plus_button_pressed();

    void on_j3_plus_button_released();

    void on_j3_minus_button_pressed();

    void on_j3_minus_button_released();

    void on_j4_plus_button_pressed();

    void on_j4_plus_button_released();

    void on_j4_minus_button_pressed();

    void on_j4_minus_button_released();

    void on_j5_plus_button_pressed();

    void on_j5_plus_button_released();

    void on_j5_minus_button_pressed();

    void on_j5_minus_button_released();

    void on_j6_plus_button_pressed();

    void on_j6_plus_button_released();

    void on_j6_minus_button_pressed();

    void on_j6_minus_button_released();

    void on_pick_line_button_2_clicked();

    void on_pick_line_button_3_clicked();

    void on_place_line_button_2_clicked();

    void on_place_line_button_3_clicked();

    void on_pick_line_button_1_clicked();

    void on_place_line_button_1_clicked();

private:
    Ui::MainWindow *ui;
    QPushButton *pick_line_button;
    QPushButton *place_line_button;
    QPushButton *pick_approach_button;
    QPushButton *pick_retreat_button;
    QPushButton *place_approach_button;
    QPushButton *place_retreat_button;
    QPushButton *stop_button;
    QPushButton *return_button;

    QPushButton *init_arm_button;
    QPushButton *init_arm_controller_button;
    QPushButton *stop_arm_button;
    QPushButton *close_arm_button;

    QPushButton *init_tool_button;
    QPushButton *cancel_tool_button;
    QPushButton *open_gripper_button;
    QPushButton *close_gripper_button;
    QPushButton *up_sleeve_button;
    QPushButton *down_sleeve_button;
    QPushButton *stop_tool_button;
    QPushButton *tighten_bolt_button;
    QPushButton *loose_bolt_button;

    QPushButton *x_plus_button;
    QPushButton *x_minus_button;
    QPushButton *y_plus_button;
    QPushButton *y_minus_button;
    QPushButton *z_plus_button;
    QPushButton *z_minus_button;
    QPushButton *rx_plus_button;
    QPushButton *rx_minus_button;
    QPushButton *ry_plus_button;
    QPushButton *ry_minus_button;
    QPushButton *rz_plus_button;
    QPushButton *rz_minus_button;

    QTextEdit *sys_info;
    QScrollBar *sys_bar;
    QString IPadress;
    QFile fp;
    thread_sort *t_sort;
    double originPoint[3]={-1.91681,-0.199768,0.702630};

    QTcpServer *server;
    QTcpSocket *socket=NULL;

    bool tool_init;
    bool arm_init;
    sample *sp;
};

#endif // MAINWINDOW_H
