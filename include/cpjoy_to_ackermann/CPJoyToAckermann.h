#ifndef CPJOYTOACKERMANN_H
#define CPJOYTOACKERMANN_H

#include <QContextMenuEvent>

#include <control_panel/ControlPanelPlugin.h>
#include <joy_to_ackermann/joy_to_ackermann_nodelet.hpp>

#include <ros/ros.h>

namespace Ui {
class CPJoyToAckermannPlugin;
}

namespace control_panel
{
class CPJoyToAckermannPlugin : public ControlPanelPlugin
{
    Q_OBJECT

public:
    explicit CPJoyToAckermannPlugin();
    ~CPJoyToAckermannPlugin();
    void start();
    void stop();
    void setup();
    boost::shared_ptr<nodelet::Nodelet> getNodelet();

public slots:
    void configDialog();
    void setActive(bool active);

protected:
    void contextMenuEvent(QContextMenuEvent *event);
    
private:
    Ui::CPJoyToAckermannPlugin *ui;
    float linear_scale;
    float angular_scale;
    joy_to_ackermann::joy_to_ackermann_nodelet *nodelet_priv;
};
}

#endif // CPJOYTOACKERMANN_H
