#ifndef THREAD_SORT
#define THREAD_SORT
#include "sortfruit.h"
#include <QtGui>
#include <stdio.h>
#include <errno.h>
#include <resolv.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include "auborobot.h"


class thread_sort: public QThread
{
    Q_OBJECT
public:
    thread_sort();
    ~thread_sort();
    void stop();
    int connect_arm();
    int connect_status();
    void fast_stop();
    void get_info(float info[9]);
    void get_action(QString act);
    void Delay(int msec);
signals:
    void fail(QString info);
    void tool_control(QString cmd);

protected:
    void run();
private:
    pthread_mutex_t stop_move_mutex = PTHREAD_MUTEX_INITIALIZER;
    float sort_info[9];
    QString action;
    SortFruit *sort;
    Eigen::Quaterniond quaternion={0.700130,-0.055469,-0.042420,-0.710593};
    Eigen::Matrix3d orimat;
    double originPoint[3]={0.058272,-0.133456,0.065710};
};


#endif // THREAD_SORT

