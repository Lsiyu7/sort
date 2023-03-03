#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>

using namespace xar;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    pick_approach_button = ui->pick_approach_button;
    pick_retreat_button = ui->pick_retreat_button;
    place_approach_button = ui->place_approach_button;
    place_retreat_button = ui->place_retreat_button;
    stop_button = ui->stop_button;
    return_button = ui->return_button;

    init_arm_button = ui->init_arm_button;
    init_arm_controller_button = ui->init_arm_controller_button;
    stop_arm_button = ui->stop_arm_button;
    close_arm_button = ui->close_arm_button;

    init_tool_button = ui->init_tool_button;
    cancel_tool_button = ui->cancel_tool_button;
    open_gripper_button = ui->open_gripper_button;
    close_gripper_button = ui->close_gripper_button;
    up_sleeve_button = ui->up_sleeve_button;
    down_sleeve_button = ui->down_sleeve_button;
    stop_tool_button = ui->stop_tool_button;
    tighten_bolt_button = ui->tighten_bolt_button;
    loose_bolt_button = ui->loost_bolt_button;

    x_plus_button = ui->x_plus_button;
    x_minus_button = ui->x_minus_button;
    y_plus_button = ui->y_plus_button;
    y_minus_button = ui->x_minus_button;
    z_plus_button = ui->z_plus_button;
    z_minus_button = ui->z_minus_button;
    rx_plus_button = ui->rx_plus_button;
    rx_minus_button = ui->rx_minus_button;
    ry_plus_button = ui->ry_plus_button;
    ry_minus_button = ui->ry_minus_button;
    rz_plus_button = ui->rz_plus_button;
    rz_minus_button = ui->rz_minus_button;

    sys_info = ui->textEdit_2;
    sys_info->setTextColor(Qt::red);
    QFont font1;
    font1.setPointSize(12);
    sys_info->setFont(font1);
    sys_bar = sys_info->verticalScrollBar();

    t_sort = new thread_sort();
    QObject::connect(t_sort,SIGNAL(fail(QString)),this,SLOT(fail_info(QString)));
    QObject::connect(t_sort,SIGNAL(tool_control(QString)),this,SLOT(tool_cmd(QString)));

    server = new QTcpServer();
    QObject::connect(server,&QTcpServer::newConnection,this,&MainWindow::server_New_Connect);

    int port=9999;
    server->listen(QHostAddress::Any, port);
    sp = new sample();
    ::connect("192.168.0.10");//plc gong kong ji
    Delay(100);
    int r = GetNetState();
    if (r == 0)
    {
        QString str1 = "系统信息：与主控通信失败. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
    }
    else
    {
        QString str1 = "系统信息：与主控通信成功. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
    }
    int a = Yx(5);
    int b = Yx(6);
    int c = Yx(7);
    qDebug() << a << b << c << "\n";
    tool_init = false;
    arm_init = false;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::server_New_Connect()
{
    socket = server->nextPendingConnection();

    QObject::connect(socket, &QTcpSocket::readyRead, this, &MainWindow::socket_Read_Data);

    qDebug() << "A Client connect!";
}

void MainWindow::socket_Read_Data()
{
    QByteArray buffer;
    //读取缓冲区数据
    QString str;
    buffer = socket->readAll();
    if(!buffer.isEmpty())
    {
        str+=tr(buffer);
        qDebug()<<str<<"\n";
    }
    QStringList list;
    list = str.split("?");
    if (list[0] == "getImage2D")
    {
        get_image();
    }
    else if (list[0] == "targetPose")
    {
        float pixel[4];
        QStringList nlist = list[1].split("&");
        for (int i=0;i<4;i++)
        {
           QStringList nnlist = nlist[i].split("=");
           pixel[i] = nnlist[1].toFloat();
           qDebug() << pixel[i] << "\n";
        }
        qDebug() << "Start recognize\n";
        recognize(pixel[0],pixel[1],pixel[2],pixel[3]);
    }
    else if (list[0] == "stop_arm")
    {
        on_stop_button_clicked();
    }
    else if (list[0] == "return")
    {
        on_return_button_clicked();
    }
    else if (list[0] == "stop_tool")
    {
        on_stop_tool_button_clicked();
    }
    else if (list[0] == "x_plus")
    {
         on_x_plus_button_pressed();
    }
    else if (list[0] == "x_minus")
    {
        on_x_minus_button_pressed();
    }
    else if (list[0] == "y_plus")
    {
         on_y_plus_button_pressed();
    }
    else if (list[0] == "y_minus")
    {
        on_y_minus_button_pressed();
    }
    else if (list[0] == "z_plus")
    {
         on_z_plus_button_pressed();
    }
    else if (list[0] == "z_minus")
    {
        on_z_minus_button_pressed();
    }
    else if (list[0] == "rx_plus")
    {
         on_rx_plus_button_pressed();
    }
    else if (list[0] == "rx_minus")
    {
        on_rx_minus_button_pressed();
    }
    else if (list[0] == "ry_plus")
    {
         on_ry_plus_button_pressed();
    }
    else if (list[0] == "ry_minus")
    {
        on_ry_minus_button_pressed();
    }
    else if (list[0] == "rz_plus")
    {
         on_rz_plus_button_pressed();
    }
    else if (list[0] == "tech_stop")
    {
         on_z_minus_button_released();
    }
    else if (list[0] == "rz_minus")
    {
        on_rz_minus_button_pressed();
    }
    else if (list[0] == "pick_approach")
    {
        on_pick_approach_button_clicked();
    }
    else if (list[0] == "pick_retreat")
    {
        on_pick_retreat_button_clicked();
    }
    else if (list[0] == "place_approach")
    {
        on_place_approach_button_clicked();
    }
    else if (list[0] == "place_retreat")
    {
        on_place_retreat_button_clicked();
    }
    else if (list[0] == "pick_line1")
    {
        on_pick_line_button_1_clicked();
    }
    else if (list[0] == "place_line1")
    {
        on_place_line_button_1_clicked();//xuyaogai
    }
    else if (list[0] == "pick_line2")
    {
        on_pick_line_button_2_clicked();
    }
    else if (list[0] == "place_line2")
    {
        on_place_line_button_2_clicked();
    }
    else if (list[0] == "pick_line3")
    {
        on_pick_line_button_3_clicked();
    }
    else if (list[0] == "place_line3")
    {
        on_place_line_button_3_clicked();
    }
    else if (list[0] == "tighten_bolt")
    {
        on_tighten_bolt_button_clicked();
    }
    else if (list[0] == "loost_bolt")
    {
        on_loost_bolt_button_clicked();
    }
    else if (list[0] == "up_sleeve")
    {
        on_up_sleeve_button_clicked();
    }
    else if (list[0] == "down_sleeve")
    {
        on_down_sleeve_button_clicked();
    }
    else if (list[0] == "close_gripper")
    {
        on_close_gripper_button_clicked();
    }
    else if (list[0] == "open_gripper")
    {
        on_open_gripper_button_clicked();
    }
    else if (list[0] == "init_tool")
    {
        on_init_tool_button_clicked();
    }
    else if (list[0] == "cancel_tool")
    {
        on_cancel_tool_button_clicked();
    }
    else if (list[0] == "init_arm")
    {
        on_init_arm_button_clicked();
    }
    else if (list[0] == "STOP_arm")
    {
        on_stop_arm_button_clicked();
    }
    else if (list[0] == "power")
    {
        on_init_arm_controller_button_clicked();
    }
    else if (list[0] == "close_arm")
    {
        on_close_arm_button_clicked();
    }
    //
    else if (list[0] == "j1_plus")
    {
        on_j1_plus_button_pressed();
    }
    else if (list[0] == "j2_plus")
    {
        on_j2_plus_button_pressed();
    }
    else if (list[0] == "j3_plus")
    {
        on_j3_plus_button_pressed();
    }
    else if (list[0] == "j4_plus")
    {
        on_j4_plus_button_pressed();
    }
    else if (list[0] == "j5_plus")
    {
        on_j5_plus_button_pressed();
    }
    else if (list[0] == "j6_plus")
    {
        on_j6_plus_button_pressed();
    }
    else if (list[0] == "j1_minus")
    {
        on_j1_minus_button_pressed();
    }
    else if (list[0] == "j2_minus")
    {
        on_j2_minus_button_pressed();
    }
    else if (list[0] == "j3_minus")
    {
        on_j3_minus_button_pressed();
    }
    else if (list[0] == "j4_minus")
    {
        on_j4_minus_button_pressed();
    }
    else if (list[0] == "j5_minus")
    {
        on_j5_minus_button_pressed();
    }
    else if (list[0] == "j6_minus")
    {
        on_j6_minus_button_pressed();
    }

}

void MainWindow::get_image()
{
    /*sp->ImgAndCloud();
    int file_block_length = 0;
    char buffer[1024];
    FILE *fp = fopen("/home/esir/Sort2/pic/gun_sun_2.jpg", "r");
    if (fp == NULL)
    {
         qDebug() << "File: test Not Found!\n";
    }*/
    bool flag = sp->ImgAndCloud();//如果保存的点云数量大于0,返回true,否则返回false
    int file_block_length = 0;
    char buffer[1024];
    FILE *fp = fopen("/home/esir/Sort2/pic/gun_sun_2.jpg", "r");
    if (flag == false || fp == NULL)
    {
         qDebug() << "cloud is null or File: test Not Found!\n";
         socket->disconnectFromHost();
    }
    else
    {
        bzero(buffer, 1024);
        while( (file_block_length = fread(buffer, sizeof(char), 1024, fp)) > 0)
        {
            if (socket->write(buffer,file_block_length)<0)
            {
                qDebug() << "Send File:test Failed!\n";
                break;
            }
            bzero(buffer, sizeof(buffer));
        }
        fclose(fp);
        socket->disconnectFromHost();
        qDebug() << "File Transfer Finished!\n";
    }
}

void MainWindow::recognize(float r1,float c1,float r2,float c2)
{
    std::vector<Eigen::Vector4f> point;
    point = sp->seg_cloud(r1,c1,r2,c2);

    Eigen::Vector4f v_1 = Eigen::Vector4f::Zero();
    Eigen::Vector4f v_2 = Eigen::Vector4f::Zero();
    Eigen::Vector4f v_3 = Eigen::Vector4f::Zero();
    std::vector<Eigen::Vector4f> v ;
    v.push_back(v_1);
    v.push_back(v_2);
    v.push_back(v_3);
    //判断分割步骤是否异常

    if(point == v){
         qDebug() << "分割点云为空";
         char str[]="no_image\n";
         char *b = str;
         try{if(socket!=NULL){
             socket->write(b,sizeof(str));
             socket->disconnectFromHost();}
             }
         catch (...){};
    }
    else{char str[]="image\n";
        char *b = str;
        try{if(socket!=NULL){
            socket->write(b,sizeof(str));
            socket->disconnectFromHost();}
            }
        catch (...){};
    }
    Eigen::Vector4f pc,p1,p2;
    pc = point[0];
    p1 = point[1];
    p2 = point[2];
    float info[9];
    info[0] = pc[0];
    info[1] = pc[1];
    info[2] = pc[2];
    info[3] = p1[0];
    info[4] = p1[1];
    info[5] = p1[2];
    info[6] = p2[0];
    info[7] = p2[1];
    info[8] = p2[2];
    t_sort->get_info(info);
    socket->disconnectFromHost();
    socket->close();
}

void MainWindow::fail_info(QString str)
{
    QByteArray ba = str.toLatin1();
    char *b = ba.data();
    try{if(socket!=NULL){
        socket->write(b,sizeof(str)*2);
        socket->disconnectFromHost();}
        }
    catch (...){};
    qDebug() << b << "\n";
}

void MainWindow::tool_cmd(QString cmd)
{
    if (cmd == "open_gripper")
    {
        on_open_gripper();
    }
    else if (cmd == "close_gripper")
    {
        on_close_gripper();
    }
    else if (cmd == "up_sleeve")
    {
        on_up_sleeve();
    }
    else if (cmd == "down_sleeve")
    {
        on_down_sleeve();
    }
    else if (cmd == "tighten_bolt")
    {
        on_tighten_bolt();
    }
    else if (cmd == "loost_bolt")
    {
        on_loost_bolt();
    }
}

void MainWindow::Delay(int msec)
{
    QEventLoop loop;
    QTimer::singleShot(msec,&loop,SLOT(quit()));
    loop.exec();
}

void MainWindow::on_stop_button_clicked()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    t_sort->stop();
    t_sort->quit();
}

void MainWindow::on_pick_approach_button_clicked()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    t_sort->get_action("pick_approach");
    t_sort->start();//开始执行线程
}

void MainWindow::on_pick_retreat_button_clicked()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    t_sort->get_action("pick_retreat");
    t_sort->start();//开始执行线程
}

void MainWindow::on_place_approach_button_clicked()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    t_sort->get_action("place_approach");
    t_sort->start();//开始执行线程
}

void MainWindow::on_place_retreat_button_clicked()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    t_sort->get_action("place_retreat");
    t_sort->start();//开始执行线程
}

void MainWindow::on_init_tool_button_clicked()
{
    init_tool_button->setEnabled(false);
    SetManualOP(1);
    Delay(300);
    SetWorkMode(48);
    Delay(300);
    PressStart();
    Delay(400);
    PressStart();
    Delay(400);
    QString str1 = "系统信息：与工具端连接成功. \n";
    sys_info->insertPlainText(str1);
    sys_bar->setSliderPosition(sys_bar->maximum());
    cancel_tool_button->setEnabled(true);
    fail_info("tool=1\n");
    tool_init = true;
}

void MainWindow::on_cancel_tool_button_clicked()
{
    init_tool_button->setEnabled(true);
    SetYx(1610,0);
    SetYx(1611,0);
    SetYx(1612,0);
    PressPause();
    Delay(300);
    SetWorkMode(51);
    Delay(1500);
    ServoEnable();
    Delay(500);
    SetManualOP(0);
    Delay(300);
    QString str1 = "系统信息：与工具端断开. \n";
    sys_info->insertPlainText(str1);
    sys_bar->setSliderPosition(sys_bar->maximum());

    fail_info("tool=0\n");
    tool_init = false;
    cancel_tool_button->setEnabled(false);
}

void MainWindow::on_stop_tool_button_clicked()
{
    SetYx(1610,0);
    SetYx(1611,0);
    SetYx(1612,0);
    fail_info("tool_stop\n");
}

void MainWindow::on_close_gripper_button_clicked()
{
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    SetYx(1610,0);
    SetYx(1611,0);
    SetYx(1612,0);
    Delay(100);

    SetYx(1612,1);
    Delay(10000);
    int v = Yx(0);
    fail_info("gripper=1\n");
}

void MainWindow::on_open_gripper_button_clicked()
{
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    SetYx(1610,0);
    SetYx(1611,0);
    SetYx(1612,0);
    Delay(100);

    SetYx(1612,2);
    int v = 0;
    int i = 0;
    while(1)
    {
        v = Yx(0);
        if (v == 1)
        {
            break;
        }
        if (i >= 15)
        {
            break;
        }
        Delay(1000);
        i++;
    }
    fail_info("gripper=0\n");
}

void MainWindow::on_up_sleeve_button_clicked()
{
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    SetYx(1610,0);
    SetYx(1611,0);
    SetYx(1612,0);
    Delay(100);

    SetYx(1611,2);
    int v = 0;
    int i = 0;
    while(1)
    {
        v = Yx(2);
        if (v == 1)
        {
            break;
        }
        if (i >= 15)
        {
            break;
        }
        Delay(1000);
        i++;
    }
    fail_info("sleeve=1\n");
}

void MainWindow::on_down_sleeve_button_clicked()
{
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    SetYx(1610,0);
    SetYx(1611,0);
    SetYx(1612,0);
    Delay(100);

    SetYx(1611,1);
    int v = 0;
    int i = 0;
    while(1)
    {
        v = Yx(1);
        if (v == 1)
        {
            break;
        }
        if (i >= 15)
        {
            break;
        }
        Delay(1000);
        i++;
    }
    fail_info("sleeve=0\n");
}

void MainWindow::on_tighten_bolt_button_clicked()
{
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    SetYx(1610,0);
    SetYx(1611,0);
    SetYx(1612,0);
    Delay(100);

    SetYx(1610,1);
    Delay(11000);
    fail_info("bolt=1\n");
}

void MainWindow::on_loost_bolt_button_clicked()
{
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    SetYx(1610,0);
    SetYx(1611,0);
    SetYx(1612,0);
    Delay(100);

    SetYx(1610,2);
    Delay(11000);
    fail_info("bolt=0\n");
}

void MainWindow::on_close_gripper()
{
     SetYx(1612,1);
}

void MainWindow::on_open_gripper()
{
    SetYx(1612,2);
}

void MainWindow::on_up_sleeve()
{
    SetYx(1611,2);
}

void MainWindow::on_down_sleeve()
{
    SetYx(1611,1);
}

void MainWindow::on_tighten_bolt()
{
    SetYx(1610,1);
}

void MainWindow::on_loost_bolt()
{
    SetYx(1610,2);
}

void MainWindow::on_init_arm_button_clicked()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    init_arm_button->setEnabled(false);
    t_sort->get_action("start_up");
    t_sort->start();
    close_arm_button->setEnabled(true);

    Delay(12000);
    fail_info("robot=1\n");
    arm_init = true;
}

void MainWindow::on_stop_arm_button_clicked()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    close_arm_button->setEnabled(false);
    t_sort->fast_stop();
    fail_info("arm_stop\n");
    arm_init = false;
    init_arm_button->setEnabled(true);
}

void MainWindow::on_close_arm_button_clicked()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    close_arm_button->setEnabled(false);
    t_sort->fast_stop();
    fail_info("close=0\n");
    arm_init = false;
    init_arm_button->setEnabled(true);
}

void MainWindow::on_return_button_clicked()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("return");
    t_sort->start();
    fail_info("return\n");
}

void MainWindow::on_init_arm_controller_button_clicked()
{
    if (t_sort->connect_status())
    {
        fail_info("connect=1\n");
        return;
    }
    int res =t_sort->connect_arm();
    if (res == 1)
    {
        QString str1 = "系统信息：与机械臂通信成功. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=1\n");
        return;
    }
    else
    {
        SetYx(1640,1);
        char str[]="power=1\n";
        char *b = str;
        try{if(socket!=NULL)
            {
            socket->write(b,sizeof(str));
//            socket->disconnectFromHost();
            }
            }
        catch (...){};
        Delay(35000);
        res =t_sort->connect_arm();
        if (res == 0)
        {
            QString str1 = "系统信息：与机械臂通信失败. \n";
            sys_info->insertPlainText(str1);
            sys_bar->setSliderPosition(sys_bar->maximum());
            char str[]="connect=0\n";
            char *b = str;
            try{if(socket!=NULL){
                socket->write(b,sizeof(str));
                socket->disconnectFromHost();}
                }
            catch (...){};
        }
        else
        {
            QString str1 = "系统信息：与机械臂通信成功. \n";
            sys_info->insertPlainText(str1);
            sys_bar->setSliderPosition(sys_bar->maximum());
            fail_info("connect=1\n");
        }
    }
}

//0316 pressed and released
void MainWindow::on_x_plus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("x_plus");
    t_sort->start();
    fail_info("x_plus\n");
}

void MainWindow::on_x_plus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_x_minus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");

        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("x_minus");
    t_sort->start();
    fail_info("x_minus\n");
}

void MainWindow::on_x_minus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_y_plus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("y_plus");
    t_sort->start();
    fail_info("y_plus\n");
}

void MainWindow::on_y_plus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_y_minus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("y_minus");
    t_sort->start();
    fail_info("y_minus\n");
}

void MainWindow::on_y_minus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_z_plus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        fail_info("connect=0\n");

        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("z_plus");
    t_sort->start();
    fail_info("z_plus\n");
}

void MainWindow::on_z_plus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_z_minus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");

        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("z_minus");
    t_sort->start();
    fail_info("z_minus\n");
}

void MainWindow::on_z_minus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_rx_plus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");

        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("rx_plus");
    t_sort->start();
        fail_info("rx_plus\n");
}

void MainWindow::on_rx_plus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
        fail_info("tech_stop\n");
}

void MainWindow::on_rx_minus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

    fail_info("connect=0\n");

        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("rx_minus");
    t_sort->start();
        fail_info("rx_minus\n");
}

void MainWindow::on_rx_minus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
        fail_info("tech_stop\n");
}

void MainWindow::on_ry_plus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("ry_plus");
    t_sort->start();
    fail_info("ry_plus\n");
}

void MainWindow::on_ry_plus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
        fail_info("tech_stop\n");
}

void MainWindow::on_ry_minus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("ry_minus");
    t_sort->start();
        fail_info("ry_minus\n");
}

void MainWindow::on_ry_minus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
        fail_info("tech_stop\n");
}

void MainWindow::on_rz_plus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("rz_plus");
    t_sort->start();
        fail_info("rx_plus\n");
}

void MainWindow::on_rz_plus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
        fail_info("tech_stop\n");
}

void MainWindow::on_rz_minus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("rz_minus");
    t_sort->start();
        fail_info("rz_minus\n");
}

void MainWindow::on_rz_minus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
        fail_info("tech_stop\n");
}

void MainWindow::on_j1_plus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("j1_plus");
    t_sort->start();
        fail_info("j1_plus\n");
}

void MainWindow::on_j1_plus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_j1_minus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("j1_minus");
    t_sort->start();
    fail_info("j1_minus\n");
}

void MainWindow::on_j1_minus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_j2_plus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("j2_plus");
    t_sort->start();
    fail_info("j2_plus\n");
}

void MainWindow::on_j2_plus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_j2_minus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("j2_minus");
    t_sort->start();
    fail_info("j2_minus\n");
}

void MainWindow::on_j2_minus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_j3_plus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("j3_plus");
    t_sort->start();
    fail_info("j3_plus\n");
}

void MainWindow::on_j3_plus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_j3_minus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("j3_minus");
    t_sort->start();
    fail_info("j3_minus\n");
}

void MainWindow::on_j3_minus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
        fail_info("tech_stop\n");
}

void MainWindow::on_j4_plus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        fail_info("connect=0\n");

        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("j4_plus");
    t_sort->start();
    fail_info("j4_plus\n");
}

void MainWindow::on_j4_plus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
        fail_info("tech_stop\n");
}

void MainWindow::on_j4_minus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("j4_minus");
    t_sort->start();
    fail_info("j4_minus\n");
}

void MainWindow::on_j4_minus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_j5_plus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("j5_plus");
    t_sort->start();
    fail_info("j5_plus\n");
}

void MainWindow::on_j5_plus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_j5_minus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("j5_minus");
    t_sort->start();
    fail_info("j5_minus\n");
}

void MainWindow::on_j5_minus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_j6_plus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("j6_plus");
    t_sort->start();
    fail_info("j6_plus\n");
}

void MainWindow::on_j6_plus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_j6_minus_button_pressed()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("connect=0\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    t_sort->get_action("j6_minus");
    t_sort->start();
        fail_info("j6_minus\n");
}

void MainWindow::on_j6_minus_button_released()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());
        return;
    }
    t_sort->get_action("tech_stop");
    t_sort->start();
    fail_info("tech_stop\n");
}

void MainWindow::on_pick_line_button_1_clicked()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("tech_stop\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    int v1 = Yx(6);
    if(v1==1)
    {
        t_sort->get_action("pick_line1");
        t_sort->start();//开始执行线程
        return;
    }
    int v2 = Yx(5);
    if(v2==1)
    {
        t_sort->get_action("pick_line2");
        t_sort->start();//开始执行线程
        return;
    }
    int v3 = Yx(7);
    if(v3==1)
    {
        t_sort->get_action("pick_line3");
        t_sort->start();//开始执行线程
        return;
    }
}

void MainWindow::on_pick_line_button_2_clicked()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("tech_stop\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    t_sort->get_action("pick_line2");
    t_sort->start();//开始执行线程
}

void MainWindow::on_pick_line_button_3_clicked()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("tech_stop\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    t_sort->get_action("pick_line3");
    t_sort->start();//开始执行线程
}

void MainWindow::on_place_line_button_1_clicked()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("tech_stop\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    int v1 = Yx(6);
    if(v1==0)
    {
        t_sort->get_action("place_line1");
        t_sort->start();//开始执行线程
        return;
    }
    int v2 = Yx(5);
    if(v2==0)
    {
        t_sort->get_action("place_line2");
        t_sort->start();//开始执行线程
        return;
    }
    int v3 = Yx(7);
    if(v3==0)
    {
        t_sort->get_action("place_line3");
        t_sort->start();//开始执行线程
        return;
    }
}

void MainWindow::on_place_line_button_2_clicked()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("tech_stop\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    t_sort->get_action("place_line2");
    t_sort->start();//开始执行线程
}

void MainWindow::on_place_line_button_3_clicked()
{
    if (!t_sort->connect_status())
    {
        QString str1 = "系统信息：没有与机械臂连接. \n";
        sys_info->insertPlainText(str1);
        sys_bar->setSliderPosition(sys_bar->maximum());

        fail_info("tech_stop\n");
        return;
    }
    if (arm_init == false)
    {
        fail_info("robot=0\n");
        return;
    }
    if (tool_init == false)
    {
        fail_info("tool_init=0\n");
        return;
    }
    t_sort->get_action("place_line3");
    t_sort->start();//开始执行线程
}
