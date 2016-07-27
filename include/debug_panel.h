#ifndef CONTROLS_PANEL_H
#define CONTROLS_PANEL_H

#include "ros/ros.h"
#include "rviz/panel.h"
#include <boost/lexical_cast.hpp>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QCheckBox>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>

namespace mtracker_gui
{

class DebugPanel: public rviz::Panel
{
Q_OBJECT
public:
  DebugPanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

private:
  void callDebugServer();

private Q_SLOTS:
  void activateDebug(bool checked);
  void setLeftWheelVelocity();
  void setRightWheelVelocity();
  void clearLeftWheel();
  void clearRightWheel();
  void switchLed1(bool checked);
  void switchLed2(bool checked);
  void switchLed3(bool checked);

private:
  QCheckBox* debug_checkbox_;

  QPushButton* led_1_button_;
  QPushButton* led_2_button_;
  QPushButton* led_3_button_;

  QPushButton* left_wheel_button_;
  QPushButton* right_wheel_button_;

  QLineEdit* left_wheel_input_;
  QLineEdit* right_wheel_input_;

  ros::NodeHandle nh_;
  ros::ServiceClient debug_srv_;

  bool led_1_;
  bool led_2_;
  bool led_3_;

  float left_wheel_velocity_;
  float right_wheel_velocity_;
};

} // end namespace mtracker_gui

#endif // CONTROLS_PANEL_H
