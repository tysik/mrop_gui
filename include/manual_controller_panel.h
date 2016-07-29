/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include <stdio.h>
#include <algorithm>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <rviz/panel.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <QCheckBox>
#include <QLineEdit>
#include <QKeyEvent>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>

namespace mrop_gui
{

class ManualControllerPanel: public rviz::Panel
{
Q_OBJECT
public:
  ManualControllerPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

private Q_SLOTS:
  void trigger(bool checked);
  void setParamsButton();
  void switchJoy(bool checked);
  void switchKeys(bool checked);

private:
  void setParams();
  void getParams();
  bool notifyParamsUpdate();
  bool verifyInputs();

  void keyPressEvent(QKeyEvent * e);
  void keyReleaseEvent(QKeyEvent * e);

private:
  QCheckBox* activate_checkbox_;

  QPushButton* left_button_;
  QPushButton* right_button_;
  QPushButton* up_button_;
  QPushButton* down_button_;
  QPushButton* rot_left_button_;
  QPushButton* rot_right_button_;
  QPushButton* joy_button_;
  QPushButton* keys_button_;
  QPushButton* set_button_;

  QLineEdit* k_v_input_;
  QLineEdit* k_w_input_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Publisher keys_pub_;
  ros::ServiceClient params_cli_;

  int keys_[6];

  // Parameters
  bool p_active_;
  bool p_use_joy_;
  bool p_use_keys_;

  double p_linear_gain_;   // Linear velocity gain
  double p_angular_gain_;  // Angular velocity gain
};

} // end namespace mrop_gui
