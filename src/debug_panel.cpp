#include "debug_panel.h"
#include "std_srvs/Empty.h"

namespace mtracker_gui
{

DebugPanel::DebugPanel(QWidget* parent)
  : rviz::Panel(parent),
    led_1_(false),
    led_2_(false),
    led_3_(false),
    left_wheel_velocity_(0.0f),
    right_wheel_velocity_(0.0f)
{
  debug_checkbox_ = new QCheckBox("Debug active");
  debug_checkbox_->setChecked(false);

  led_1_button_ = new QPushButton("LED 1");
  led_2_button_ = new QPushButton("LED 2");
  led_3_button_ = new QPushButton("LED 3");

  led_1_button_->setCheckable(true);
  led_1_button_->setMaximumSize(50, 50);
  led_1_button_->setMinimumSize(50, 50);

  led_2_button_->setCheckable(true);
  led_2_button_->setMaximumSize(50, 50);
  led_2_button_->setMinimumSize(50, 50);

  led_3_button_->setCheckable(true);
  led_3_button_->setMaximumSize(50, 50);
  led_3_button_->setMinimumSize(50, 50);

  QHBoxLayout* led_buttons_layout = new QHBoxLayout;
  led_buttons_layout->addWidget(led_1_button_);
  led_buttons_layout->addWidget(led_2_button_);
  led_buttons_layout->addWidget(led_3_button_);

  left_wheel_input_  = new QLineEdit("0.0");
  right_wheel_input_ = new QLineEdit("0.0");

  left_wheel_input_->setToolTip("Type a value and hit enter");
  right_wheel_input_->setToolTip("Type a value and hit enter");

  left_wheel_button_  = new QPushButton("Clear");
  right_wheel_button_ = new QPushButton("Clear");

  QGridLayout *wheels_values_layout = new QGridLayout;
  wheels_values_layout->addWidget(new QLabel("Left wheel:"), 0, 0);
  wheels_values_layout->addWidget(left_wheel_input_, 0, 1);
  wheels_values_layout->addWidget(new QLabel("rad/s"), 0, 2);
  wheels_values_layout->addWidget(left_wheel_button_, 0, 3);
  wheels_values_layout->addWidget(new QLabel("Right wheel:"), 1, 0);
  wheels_values_layout->addWidget(right_wheel_input_, 1, 1);
  wheels_values_layout->addWidget(new QLabel("rad/s"), 1, 2);
  wheels_values_layout->addWidget(right_wheel_button_, 1, 3);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(debug_checkbox_);
  layout->addLayout(led_buttons_layout);
  layout->addLayout(wheels_values_layout);
  setLayout(layout);

  connect(debug_checkbox_, SIGNAL(clicked(bool)), this, SLOT(activateDebug(bool)));
  connect(left_wheel_input_, SIGNAL(returnPressed()), this, SLOT(setLeftWheelVelocity()));
  connect(right_wheel_input_, SIGNAL(returnPressed()), this, SLOT(setRightWheelVelocity()));
  connect(left_wheel_button_, SIGNAL(clicked()), this, SLOT(clearLeftWheel()));
  connect(right_wheel_button_, SIGNAL(clicked()), this, SLOT(clearRightWheel()));
  connect(led_1_button_, SIGNAL(clicked(bool)), this, SLOT(switchLed1(bool)));
  connect(led_2_button_, SIGNAL(clicked(bool)), this, SLOT(switchLed2(bool)));
  connect(led_3_button_, SIGNAL(clicked(bool)), this, SLOT(switchLed3(bool)));

  activateDebug(false);
}

void DebugPanel::callDebugServer()
{
  std_srvs::Empty my_call;
  if (ros::ok() && debug_srv_.exists())
    debug_srv_.call(my_call);
}

void DebugPanel::activateDebug(bool checked)
{
  if (checked)
    debug_srv_ = nh_.serviceClient<std_srvs::Empty>("/debug_server");
  else
    debug_srv_.shutdown();

  led_1_button_->setEnabled(checked);
  led_2_button_->setEnabled(checked);
  led_3_button_->setEnabled(checked);

  left_wheel_input_->setEnabled(checked);
  right_wheel_input_->setEnabled(checked);

  left_wheel_button_->setEnabled(checked);
  right_wheel_button_->setEnabled(checked);
}

void DebugPanel::setLeftWheelVelocity()
{
  try
  {
    left_wheel_velocity_ = boost::lexical_cast<float>(left_wheel_input_->text().toStdString());
  }
  catch(boost::bad_lexical_cast &)
  {
    left_wheel_velocity_ = 0.0f;
    left_wheel_input_->setText("0.0");
  }

  callDebugServer();
  left_wheel_input_->clearFocus();
}

void DebugPanel::setRightWheelVelocity()
{
  try
  {
    right_wheel_velocity_ = boost::lexical_cast<float>(right_wheel_input_->text().toStdString());
  }
  catch(boost::bad_lexical_cast &)
  {
    right_wheel_velocity_ = 0.0f;
    right_wheel_input_->setText("0.0");
  }

  callDebugServer();
  right_wheel_input_->clearFocus();
}

void DebugPanel::clearLeftWheel()
{
  left_wheel_velocity_ = 0.0f;
  left_wheel_input_->setText("0.0");
  callDebugServer();
}

void DebugPanel::clearRightWheel()
{
  right_wheel_velocity_ = 0.0f;
  right_wheel_input_->setText("0.0");
  callDebugServer();
}

void DebugPanel::switchLed1(bool checked)
{
  led_1_ = checked;
  callDebugServer();
}

void DebugPanel::switchLed2(bool checked)
{
  led_2_ = checked;
  callDebugServer();
}

void DebugPanel::switchLed3(bool checked)
{
  led_3_ = checked;
  callDebugServer();
}

void DebugPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void DebugPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::DebugPanel, rviz::Panel)
