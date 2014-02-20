#include "cpkeydriveackermann/CPKeyDriveAckermann.h"
#include "ui_CPKeyDriveAckermann.h"
#include <pluginlib/class_list_macros.h>
#include <QMenu>
#include <QDialog>
#include <QPushButton>
#include <QLineEdit>
#include <QSpinBox>

PLUGINLIB_EXPORT_CLASS(control_panel::CPKeyDriveAckermannPlugin, control_panel::ControlPanelPlugin)

namespace control_panel
{
CPKeyDriveAckermannPlugin::CPKeyDriveAckermannPlugin() :
    ui(new Ui::CPKeyDriveAckermannPlugin),
    topic("ackermann_cmd"),
    nodelet_priv(new CPKeyDriveAckermannNodelet(this)),
    speed(0.2),
    steering_angle(0.785),
    w(false),
    a(false),
    s(false),
    d(false)
{
    ui->setupUi(this);
    ui->label->setText("CP Key Drive Ackermann");
    ui->label->setEnabled(false);
    pub_timer.setInterval(50);
    pub_timer.moveToThread(&pub_thread);
    connect(&pub_timer, SIGNAL(timeout()), this, SLOT(timerCB()));
    connect(&pub_thread, SIGNAL(started()), &pub_timer, SLOT(start()));
}

CPKeyDriveAckermannPlugin::~CPKeyDriveAckermannPlugin()
{
    pub_timer.stop();
    pub_thread.quit();
    pub_thread.wait();
    delete ui;
}

void CPKeyDriveAckermannPlugin::start()
{
    if(!ros::ok())
    {
        ROS_WARN_STREAM("Tried to start a nodelet that wasn't ready!");
        return;
    }
    settings->setValue(uuid.toString() + "/Active", true);
    nodelet_priv->activate(topic.toStdString());
    emit setKeyCB(this, true);
    pub_thread.start();
    ui->label->setEnabled(true);
}

void CPKeyDriveAckermannPlugin::stop()
{
    ui->label->setEnabled(false);
    nodelet_priv->deactivate();
    pub_timer.stop();
    pub_thread.quit();
    pub_thread.wait();
    emit setKeyCB(this, false);
    settings->setValue(uuid.toString() + "/Active", false);
}

void CPKeyDriveAckermannPlugin::setup()
{
    ui->label->setText(settings->value(uuid.toString() + "/Label", ui->label->text()).toString());
    topic = settings->value(uuid.toString() + "/Topic", topic).toString();
    speed = settings->value(uuid.toString() + "/Speed", speed).toDouble();
    steering_angle = settings->value(uuid.toString() + "/SteeringAngle", steering_angle).toDouble();
    pub_timer.setInterval(1000.0 / settings->value(uuid.toString() + "/PublishRate", 1000.0 / pub_timer.interval()).toDouble());
    if(settings->value(uuid.toString() + "/Active", false).toBool())
        start();
}

boost::shared_ptr<nodelet::Nodelet> CPKeyDriveAckermannPlugin::getNodelet()
{
    return boost::shared_ptr<nodelet::Nodelet>(nodelet_priv);
}

void CPKeyDriveAckermannPlugin::setActive(bool active)
{
    if(active)
        start();
    else
        stop();
}

void CPKeyDriveAckermannPlugin::keyDownCB(QKeyEvent *event)
{
    pub_mutex.lock();
    switch(event->key())
    {
    case 'W':
        w = true;
        pub_ackermann.speed = s ? 0.0 : speed;
        break;
    case 'A':
        a = true;
        pub_ackermann.steering_angle = d ? 0.0 : steering_angle;
        break;
    case 'S':
        s = true;
        pub_ackermann.speed = w ? 0.0 : -speed;
        break;
    case 'D':
        d = true;
        pub_ackermann.steering_angle = a ? 0.0 : -steering_angle;
        break;
    default:
        pub_mutex.unlock();
        return;
        break;
    }
    pub_mutex.unlock();
}

void CPKeyDriveAckermannPlugin::keyUpCB(QKeyEvent *event)
{
    pub_mutex.lock();
    switch(event->key())
    {
    case 'W':
        w = false;
        pub_ackermann.speed = s ? -speed : 0.0;
        break;
    case 'A':
        a = false;
        pub_ackermann.steering_angle = d ? -steering_angle : 0.0;
        break;
    case 'S':
        s = false;
        pub_ackermann.speed = w ? speed : 0.0;
        break;
    case 'D':
        d = false;
        pub_ackermann.steering_angle = a ? steering_angle : 0.0;
        break;
    default:
        pub_mutex.unlock();
        return;
        break;
    }
    pub_mutex.unlock();
}

void CPKeyDriveAckermannPlugin::timerCB()
{
    pub_mutex.lock();
    nodelet_priv->publishMessage(pub_ackermann);
    pub_mutex.unlock();
}

void CPKeyDriveAckermannPlugin::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu menu;
    menu.addAction("Enabled");
    menu.actions()[0]->setCheckable(true);
    menu.actions()[0]->setChecked(nodelet_priv->isActive());
    connect(menu.actions()[0], SIGNAL(toggled(bool)), this, SLOT(setActive(bool)));
    menu.addAction("Configure", this, SLOT(configDialog()));
    menu.addAction("Delete", this, SLOT(delete_self()));
    menu.exec(event->globalPos());
}

void CPKeyDriveAckermannPlugin::configDialog()
{
    QDialog dialog;
    QGridLayout *layout = new QGridLayout;

    QLabel *topictxt = new QLabel(tr("Topic Name:"));
    QLineEdit *topicedit = new QLineEdit(topic);
    layout->addWidget(topictxt, 0, 0);
    layout->addWidget(topicedit, 0, 1);

    QLabel *labeltxt = new QLabel(tr("Label:"));
    QLineEdit *labeledit = new QLineEdit(ui->label->text());
    layout->addWidget(labeltxt, 1, 0);
    layout->addWidget(labeledit, 1, 1);

    QLabel *linveltxt = new QLabel(tr("Speed:"));
    QLineEdit *linveledit = new QLineEdit(QString::number(speed));
    layout->addWidget(linveltxt, 2, 0);
    layout->addWidget(linveledit, 2, 1);

    QLabel *angveltxt = new QLabel(tr("Steering Angle:"));
    QLineEdit *angveledit = new QLineEdit(QString::number(steering_angle));
    layout->addWidget(angveltxt, 3, 0);
    layout->addWidget(angveledit, 3, 1);

    QLabel *ratetxt = new QLabel(tr("Publish Rate (Hz):"));
    QSpinBox *rateedit = new QSpinBox();
    rateedit->setMinimum(1);
    rateedit->setMaximum(100);
    rateedit->setValue(1000.0 / pub_timer.interval());
    layout->addWidget(ratetxt, 4, 0);
    layout->addWidget(rateedit, 4, 1);

    QPushButton *okbutton = new QPushButton(tr("&OK"));
    layout->addWidget(okbutton, 5, 1);

    dialog.setLayout(layout);

    dialog.setWindowTitle("Plugin Configuration - Key DriveAckermann");

    connect(okbutton, SIGNAL(clicked()), &dialog, SLOT(accept()));

    if(!dialog.exec())
        return;

    if(topic != topicedit->text())
    {
        topic = topicedit->text();
        settings->setValue(uuid.toString() + "/Topic", topic);
        nodelet_priv->activate(topic.toStdString(), true);
    }

    if(ui->label->text() != labeledit->text())
    {
        ui->label->setText(labeledit->text());
        settings->setValue(uuid.toString() + "/Label", labeledit->text());
    }

    if(linveledit->text().toDouble() != speed)
    {
        pub_mutex.lock();
        speed = linveledit->text().toDouble();
        if(w && !s)
            pub_ackermann.speed = speed;
        else if(!w && s)
            pub_ackermann.speed = -speed;
        pub_mutex.unlock();
        settings->setValue(uuid.toString() + "/Speed", speed);
    }

    if(angveledit->text().toDouble() != steering_angle)
    {
        pub_mutex.lock();
        steering_angle = angveledit->text().toDouble();
        if(a && !d)
            pub_ackermann.steering_angle = steering_angle;
        else if(!a && d)
            pub_ackermann.steering_angle = -steering_angle;
        pub_mutex.unlock();
        settings->setValue(uuid.toString() + "/SteeringAngle", steering_angle);
    }

    if(rateedit->value() != 1000.0 / pub_timer.interval())
    {
        pub_timer.setInterval(1000.0 / rateedit->value());
        settings->setValue(uuid.toString() + "/PublishRate", rateedit->value());
    }
}

CPKeyDriveAckermannNodelet::CPKeyDriveAckermannNodelet(CPKeyDriveAckermannPlugin *_parent)
    : parent(_parent)
{
}

CPKeyDriveAckermannNodelet::~CPKeyDriveAckermannNodelet()
{
    ackermann_cmd.shutdown();
}

void CPKeyDriveAckermannNodelet::onInit()
{
}

void CPKeyDriveAckermannNodelet::activate(const std::string topic, bool passive)
{
    if(ackermann_cmd)
    {
        if(ackermann_cmd.getTopic() == topic)
            return;
        ackermann_cmd.shutdown();
        ros::NodeHandle nh = getNodeHandle();
        ackermann_cmd = nh.advertise<ackermann_msgs::AckermannDrive>(topic, 1, false);
    }
    else if(!passive)
    {
        ros::NodeHandle nh = getNodeHandle();
        ackermann_cmd = nh.advertise<ackermann_msgs::AckermannDrive>(topic, 1, false);
    }
}

void CPKeyDriveAckermannNodelet::deactivate()
{
    ackermann_cmd.shutdown();
}

bool CPKeyDriveAckermannNodelet::isActive()
{
    return (ackermann_cmd);
}

bool CPKeyDriveAckermannNodelet::publishMessage(const ackermann_msgs::AckermannDrive &msg)
{
    if(!ackermann_cmd)
        return false;
    ackermann_cmd.publish(ackermann_msgs::AckermannDrivePtr(new ackermann_msgs::AckermannDrive(msg)));
    return true;
}
}
