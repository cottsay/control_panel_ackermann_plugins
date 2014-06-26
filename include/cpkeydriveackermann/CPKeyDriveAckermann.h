#ifndef CPKEYDRIVEACKERMANN_H
#define CPKEYDRIVEACKERMANN_H

#include <QContextMenuEvent>
#include <QThread>
#include <QTimer>
#include <QMutex>

#include <nodelet/nodelet.h>
#include <control_panel/ControlPanelPlugin.h>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>

#include <boost/signals2/mutex.hpp>

namespace Ui {
class CPKeyDriveAckermannPlugin;
}

namespace control_panel
{
class CPKeyDriveAckermannPlugin;

class CPKeyDriveAckermannNodelet : public nodelet::Nodelet
{
public:
    CPKeyDriveAckermannNodelet(CPKeyDriveAckermannPlugin *_parent);
    ~CPKeyDriveAckermannNodelet();
    void activate(const std::string topic, bool passive = false);
    void deactivate();
    bool isActive();
    void onInit();
    bool publishMessage(const ackermann_msgs::AckermannDrive &msg);
private:
    ros::Publisher ackermann_cmd;
    CPKeyDriveAckermannPlugin *parent;
};

class CPKeyDriveAckermannPlugin : public ControlPanelPlugin
{
    Q_OBJECT

public:
    explicit CPKeyDriveAckermannPlugin();
    ~CPKeyDriveAckermannPlugin();
    void start();
    void stop();
    void setup();
    boost::shared_ptr<nodelet::Nodelet> getNodelet();

public slots:
    void configDialog();
    void setActive(bool active);
    void keyDownCB(QKeyEvent *event);
    void keyUpCB(QKeyEvent *event);
    void timerCB();

signals:
    void changeLabel(const QString &);
    void changeEnabled(bool);

protected:
    void contextMenuEvent(QContextMenuEvent *event);

    Ui::CPKeyDriveAckermannPlugin *ui;
    QString topic;
    QString name;
    CPKeyDriveAckermannNodelet *nodelet_priv;
    double steering_angle;
    double speed;
    QThread pub_thread;
    QTimer pub_timer;
    QMutex pub_mutex;
    ackermann_msgs::AckermannDrive pub_ackermann;
    bool w;
    bool a;
    bool s;
    bool d;

friend class CPKeyDriveAckermannNodelet;
};
}

#endif // CPACKERMANN_H
