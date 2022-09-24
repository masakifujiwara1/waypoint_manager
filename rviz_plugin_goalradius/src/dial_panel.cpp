#include <rviz_plugin_goalradius/dial_panel.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

#include "ui_dial_panel.h"

bool flag = false;

namespace rviz_plugin_goalradius
{
  DialPanel::DialPanel(QWidget *parent) : Panel(parent), ui_(new Ui::DialUI())
  {
    ui_->setupUi(this);
  }

  DialPanel::~DialPanel() = default;

  void DialPanel::onInitialize()
  {
    connect(ui_->dial, SIGNAL(valueChanged(int)), this, SLOT(dialValueChanged(int)));

    connect(ui_->checkBox, SIGNAL(stateChanged(int)), this, SLOT(checkstateChanged(int)));

    ui_->lineEdit->setPlaceholderText("Input value (Default : 1.0)");

    connect(ui_->lineEdit, SIGNAL(textChanged(const QString &)), this, SLOT(lineEditChanged()));
    connect(ui_->pushButton, SIGNAL(clicked()), this, SLOT(buttonClicked()));

    // pub_ = nh_.advertise<std_msgs::Float64>("dial", 1);
    goal_radius_pub = nh_.advertise<std_msgs::Float64>("waypoint_manager/set_goal_radius", 1);
    parentWidget()->setVisible(true);
  }

  void DialPanel::onEnable()
  {
    show();
    parentWidget()->show();
  }

  void DialPanel::onDisable()
  {
    hide();
    parentWidget()->hide();
  }

  void DialPanel::lineEditChanged()
  {
    std::string value_str;
    // float old_line_value = value_call;
    if (ui_->lineEdit->text().isEmpty())
      value_line = 1.0;
    else
    {
      value_str = ui_->lineEdit->text().toStdString();
      value_line = atof(value_str.c_str());
    }

    ROS_INFO("You set the goal radius value : %f", value_line);
  }

  void DialPanel::checkstateChanged(int state)
  {
    if (state == 2)
      flag = true;
    else
      flag = false;
    ROS_INFO("state: %d", state);
  }

  // void DialPanel::lineEditChanged()
  // {
  //   std::string old_topic_name = topic_name_;
  //   if (ui_->line_edit->text().isEmpty())
  //     topic_name_ = "dial";
  //   else
  //     topic_name_ = ui_->line_edit->text().toStdString();

  //   ROS_INFO("You set the topic name : %s", topic_name_.c_str());

  //   if (old_topic_name != topic_name_)
  //     pub_ = nh_.advertise<std_msgs::Float64>(topic_name_, 1);
  // }

  void DialPanel::dialValueChanged(int value)
  {
    float value_;
    value_ = value * 0.01;
    ui_->lcd->display(value_);
    value_call = value;
    // ROS_INFO("You set the value : %f", value_);
  }

  void DialPanel::buttonClicked()
  {
    std_msgs::Float64 msg;
    if (flag == false)
      msg.data = static_cast<double>(value_call);
    else
      msg.data = static_cast<double>(value_line * 100);
    // pub_.publish(msg);
    goal_radius_pub.publish(msg);
    ROS_INFO("You pushed the button.");
  }

} // namespace rviz_plugin_goalradius

PLUGINLIB_EXPORT_CLASS(rviz_plugin_goalradius::DialPanel, rviz::Panel)
