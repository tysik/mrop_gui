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

#include "manual_controller_panel.h"

namespace mtracker_gui
{

ManualControllerPanel::ManualControllerPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), k_v_(0.4), k_w_(1.5) {
  trigger_cli_ = nh_.serviceClient<mtracker::Trigger>("manual_controller_trigger_srv");
  params_cli_ = nh_.serviceClient<mtracker::Params>("manual_controller_params_srv");
  keys_pub_ = nh_.advertise<geometry_msgs::Twist>("keys", 10);

  std::fill_n(keys_, 4, 0.0);

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(false);

  joy_button_   = new QPushButton("Joy");
  keys_button_  = new QPushButton("Keys");
  up_button_    = new QPushButton("W");
  left_button_  = new QPushButton("A");
  down_button_  = new QPushButton("S");
  right_button_ = new QPushButton("D");
  set_button_   = new QPushButton("Set");

  joy_button_->setCheckable(true);
  joy_button_->setMinimumSize(50, 50);
  joy_button_->setMaximumSize(50, 50);
  joy_button_->setEnabled(false);

  keys_button_->setCheckable(true);
  keys_button_->setMinimumSize(50, 50);
  keys_button_->setMaximumSize(50, 50);
  keys_button_->setEnabled(false);

  up_button_->setMinimumSize(50, 50);
  up_button_->setMaximumSize(50, 50);
  up_button_->setEnabled(false);

  down_button_->setMinimumSize(50, 50);
  down_button_->setMaximumSize(50, 50);
  down_button_->setEnabled(false);

  left_button_->setMinimumSize(50, 50);
  left_button_->setMaximumSize(50, 50);
  left_button_->setEnabled(false);

  right_button_->setMinimumSize(50, 50);
  right_button_->setMaximumSize(50, 50);
  right_button_->setEnabled(false);

  set_button_->setEnabled(false);

  k_v_input_ = new QLineEdit("0.4");
  k_v_input_->setEnabled(false);

  k_w_input_ = new QLineEdit("1.5");
  k_w_input_->setEnabled(false);

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QGridLayout* buttons_layout = new QGridLayout;
  buttons_layout->addItem(margin, 0, 0, 2, 1);
  buttons_layout->addWidget(joy_button_, 0, 1, Qt::AlignLeft);
  buttons_layout->addWidget(up_button_, 0, 3, Qt::AlignCenter);
  buttons_layout->addWidget(keys_button_, 0, 5, Qt::AlignRight);
  buttons_layout->addItem(margin, 0, 6, 2, 1);
  buttons_layout->addWidget(left_button_, 1, 2, Qt::AlignRight);
  buttons_layout->addWidget(down_button_, 1, 3, Qt::AlignCenter);
  buttons_layout->addWidget(right_button_, 1, 4, Qt::AlignLeft);

  QHBoxLayout* gains_layout = new QHBoxLayout;
  gains_layout->addWidget(new QLabel("k_v:"));
  gains_layout->addWidget(k_v_input_);
  gains_layout->addWidget(new QLabel("m/s"));
  gains_layout->addItem(new QSpacerItem(10, 1));
  gains_layout->addWidget(new QLabel("k_w:"));
  gains_layout->addWidget(k_w_input_);
  gains_layout->addWidget(new QLabel("rad/s"));

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addLayout(buttons_layout);
  layout->addLayout(gains_layout);
  layout->addWidget(set_button_);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked(bool)), this, SLOT(trigger(bool)));
  connect(joy_button_, SIGNAL(clicked()), this, SLOT(updateParams()));
  connect(keys_button_, SIGNAL(clicked(bool)), this, SLOT(switchKeys(bool)));
  connect(set_button_, SIGNAL(clicked()), this, SLOT(updateParams()));
}

void ManualControllerPanel::trigger(bool checked) {
  mtracker::Trigger trigger;
  trigger.request.activate = checked;

  if (trigger_cli_.call(trigger)) {
    if (checked) {
      joy_button_->setEnabled(true);
      keys_button_->setEnabled(true);
      set_button_->setEnabled(true);
      k_v_input_->setEnabled(true);
      k_w_input_->setEnabled(true);
    }
    else {
      joy_button_->setEnabled(false);
      keys_button_->setEnabled(false);
      set_button_->setEnabled(false);
      up_button_->setEnabled(false);
      down_button_->setEnabled(false);
      left_button_->setEnabled(false);
      right_button_->setEnabled(false);
      k_v_input_->setEnabled(false);
      k_w_input_->setEnabled(false);

      std::fill_n(keys_, 4, 0.0);
      keys_pub_.publish(geometry_msgs::Twist());
    }
  }
  else {
    activate_checkbox_->setChecked(!checked);
  }
}

void ManualControllerPanel::updateParams() {
  mtracker::Params params;
  params.request.params.resize(4, 0.0);

  if (!verifyInputs())
    return;

  params.request.params[0] = k_v_;
  params.request.params[1] = k_w_;

  if (joy_button_->isChecked())
    params.request.params[2] = 1.0;

  if (keys_button_->isChecked())
    params.request.params[3] = 1.0;

  params_cli_.call(params);
}

void ManualControllerPanel::switchKeys(bool checked) {
  if (checked) {
    up_button_->setEnabled(true);
    down_button_->setEnabled(true);
    left_button_->setEnabled(true);
    right_button_->setEnabled(true);
  }
  else {
    up_button_->setEnabled(false);
    down_button_->setEnabled(false);
    left_button_->setEnabled(false);
    right_button_->setEnabled(false);

    std::fill_n(keys_, 4, 0.0);
    keys_pub_.publish(geometry_msgs::Twist());
  }

  updateParams();
}

bool ManualControllerPanel::verifyInputs() {
  try { k_v_ = boost::lexical_cast<double>(k_v_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { k_v_ = 0.0; k_v_input_->setText(":-("); return false; }

  try { k_w_ = boost::lexical_cast<double>(k_w_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { k_w_ = 0.0; k_w_input_->setText(":-("); return false; }

  return true;
}

void ManualControllerPanel::keyPressEvent(QKeyEvent * e) {
  if (keys_button_->isChecked()) {
    geometry_msgs::Twist keys;

    if (!verifyInputs())
      return;

    if (e->key() == Qt::Key_W || e->key() == Qt::Key_Up) {
      up_button_->setDown(true);
      keys_[0] = 1;
    }
    if (e->key() == Qt::Key_A || e->key() == Qt::Key_Left) {
      left_button_->setDown(true);
      keys_[1] = 1;
    }
    if (e->key() == Qt::Key_S || e->key() == Qt::Key_Down) {
      down_button_->setDown(true);
      keys_[2] = 1;
    }
    if (e->key() == Qt::Key_D || e->key() == Qt::Key_Right) {
      right_button_->setDown(true);
      keys_[3] = 1;
    }

    keys.linear.x = k_v_ * (keys_[0] - keys_[2]);
    keys.angular.z = k_w_ * (keys_[1] - keys_[3]);

    keys_pub_.publish(keys);
  }
}

void ManualControllerPanel::keyReleaseEvent(QKeyEvent * e) {
  if (keys_button_->isChecked()) {
    geometry_msgs::Twist keys;

    if (!verifyInputs())
      return;

    if (e->key() == Qt::Key_W || e->key() == Qt::Key_Up) {
      up_button_->setDown(false);
      keys_[0] = 0;
    }
    if (e->key() == Qt::Key_A || e->key() == Qt::Key_Left) {
      left_button_->setDown(false);
      keys_[1] = 0;
    }
    if (e->key() == Qt::Key_S || e->key() == Qt::Key_Down) {
      down_button_->setDown(false);
      keys_[2] = 0;
    }
    if (e->key() == Qt::Key_D || e->key() == Qt::Key_Right) {
      right_button_->setDown(false);
      keys_[3] = 0;
    }

    keys.linear.x = k_v_ * (keys_[0] - keys_[2]);
    keys.angular.z = k_w_ * (keys_[1] - keys_[3]);

    keys_pub_.publish(keys);
  }
}


void ManualControllerPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ManualControllerPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::ManualControllerPanel, rviz::Panel)
