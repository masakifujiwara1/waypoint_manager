#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <rviz/panel.h>

namespace Ui
{
  class DialUI;
}

namespace rviz_plugin_goalradius
{
  class DialPanel : public rviz::Panel
  {
    Q_OBJECT
  public:
    DialPanel(QWidget *parent = nullptr);
    ~DialPanel() override;

    void onInitialize() override;
    void onEnable();
    void onDisable();

  private Q_SLOTS:
    void dialValueChanged(int value);
    void checkstateChanged(int state);
    void lineEditChanged();
    void buttonClicked();

  protected:
    Ui::DialUI *ui_;
    int value_call{0};
    float value_line{1};
    std::string topic_name_{"dial"};

    ros::NodeHandle nh_;
    ros::Publisher pub_,
        goal_radius_pub;
    // ros::ServiceClient goal_radius_client;
  };
} // end namespace rviz_plugin_goalradius
