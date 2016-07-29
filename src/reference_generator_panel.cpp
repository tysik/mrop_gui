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

#include "reference_generator_panel.h"

namespace mrop_gui
{

ReferenceGeneratorPanel::ReferenceGeneratorPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), nh_local_("reference_generator") {
  params_cli_ = nh_local_.serviceClient<std_srvs::Empty>("params");
  getParams();

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(p_active_);

  stop_button_ = new QPushButton();
  pause_button_ = new QPushButton();
  play_button_ = new QPushButton();

  set_button_ = new QPushButton("Set");
  set_button_->setEnabled(p_active_);

  stop_button_->setMinimumSize(50, 50);
  stop_button_->setMaximumSize(50, 50);
  stop_button_->setCheckable(true);
  stop_button_->setEnabled(p_active_ && !p_stopped_);
  stop_button_->setChecked(p_stopped_ && !p_paused_);
  stop_button_->setIcon(QIcon("/home/ksis/catkin_ws/src/mrop_gui/resources/stop.png"));
  stop_button_->setIconSize(QSize(25, 25));

  pause_button_->setMinimumSize(50, 50);
  pause_button_->setMaximumSize(50, 50);
  pause_button_->setCheckable(true);
  pause_button_->setEnabled(p_active_ && !p_paused_);
  pause_button_->setChecked(p_paused_ && !p_stopped_);
  pause_button_->setIcon(QIcon("/home/ksis/catkin_ws/src/mrop_gui/resources/pause.png"));
  pause_button_->setIconSize(QSize(25, 25));

  play_button_->setMinimumSize(50, 50);
  play_button_->setMaximumSize(50, 50);
  play_button_->setCheckable(true);
  play_button_->setEnabled(p_active_ && (p_stopped_ || p_paused_));
  play_button_->setChecked(!p_paused_ && !p_stopped_);
  play_button_->setIcon(QIcon("/home/ksis/catkin_ws/src/mrop_gui/resources/play.png"));
  play_button_->setIconSize(QSize(25, 25));

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QHBoxLayout* buttons_layout = new QHBoxLayout;
  buttons_layout->addSpacerItem(margin);
  buttons_layout->addWidget(stop_button_);
  buttons_layout->addWidget(pause_button_);
  buttons_layout->addWidget(play_button_);
  buttons_layout->addSpacerItem(margin);

  trajectories_list_ = new QComboBox();
  trajectories_list_->addItem("Point");
  trajectories_list_->addItem("Linear");
  trajectories_list_->addItem("Harmonic");
  trajectories_list_->addItem("Lemniscate");
  trajectories_list_->setEnabled(p_active_);
  trajectories_list_->setCurrentIndex(p_trajectory_type_);

  x_input_ = new QLineEdit(QString::number(p_x_0_));
  y_input_ = new QLineEdit(QString::number(p_y_0_));
  theta_input_ = new QLineEdit(QString::number(p_theta_0_));
  v_input_ = new QLineEdit(QString::number(p_v_));
  T_input_ = new QLineEdit(QString::number(p_T_));
  r_x_input_ = new QLineEdit(QString::number(p_r_x_));
  r_y_input_ = new QLineEdit(QString::number(p_r_y_));
  n_x_input_ = new QLineEdit(QString::number(p_n_x_));
  n_y_input_ = new QLineEdit(QString::number(p_n_y_));

  chooseTrajectoryParams(p_trajectory_type_);

  QString theta = QChar(0x03B8);

  QGridLayout* inputs_layout = new QGridLayout;
  inputs_layout->addItem(margin, 0, 0);
  inputs_layout->addWidget(new QLabel("x:"), 0, 1);
  inputs_layout->addWidget(x_input_, 0, 2);
  inputs_layout->addWidget(new QLabel("m, "), 0, 3);
  inputs_layout->addWidget(new QLabel("y:"), 0, 4);
  inputs_layout->addWidget(y_input_, 0, 5);
  inputs_layout->addWidget(new QLabel("m, "), 0, 6);
  inputs_layout->addWidget(new QLabel(theta + ":"), 0, 7);
  inputs_layout->addWidget(theta_input_, 0, 8);
  inputs_layout->addWidget(new QLabel("rad"), 0, 9);
  inputs_layout->addItem(margin, 0, 10);
  //
  inputs_layout->addItem(margin, 1, 0);
  inputs_layout->addWidget(new QLabel("v:"), 1, 1);
  inputs_layout->addWidget(v_input_, 1, 2);
  inputs_layout->addWidget(new QLabel("m/s, "), 1, 3);
  inputs_layout->addWidget(new QLabel("n<sub>x</sub>:"), 1, 4);
  inputs_layout->addWidget(n_x_input_, 1, 5);
  inputs_layout->addWidget(new QLabel("-, "), 1, 6);
  inputs_layout->addWidget(new QLabel("n<sub>y</sub>:"), 1, 7);
  inputs_layout->addWidget(n_y_input_, 1, 8);
  inputs_layout->addWidget(new QLabel("-"), 1, 9);
  inputs_layout->addItem(margin, 1, 10);
  //
  inputs_layout->addItem(margin, 2, 0);
  inputs_layout->addWidget(new QLabel("T:"), 2, 1);
  inputs_layout->addWidget(T_input_, 2, 2);
  inputs_layout->addWidget(new QLabel("s, "), 2, 3);
  inputs_layout->addWidget(new QLabel("r<sub>x</sub>:"), 2, 4);
  inputs_layout->addWidget(r_x_input_, 2, 5);
  inputs_layout->addWidget(new QLabel("m, "), 2, 6);
  inputs_layout->addWidget(new QLabel("r<sub>y</sub>:"), 2, 7);
  inputs_layout->addWidget(r_y_input_, 2, 8);
  inputs_layout->addWidget(new QLabel("m"), 2, 9);
  inputs_layout->addItem(margin, 2, 10);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addLayout(buttons_layout);
  layout->addWidget(trajectories_list_);
  layout->addLayout(inputs_layout);
  layout->addWidget(set_button_);
  layout->setAlignment(layout, Qt::AlignCenter);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked(bool)), this, SLOT(trigger(bool)));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stop()));
  connect(pause_button_, SIGNAL(clicked()), this, SLOT(pause()));
  connect(play_button_, SIGNAL(clicked()), this, SLOT(start()));
  connect(set_button_, SIGNAL(clicked()), this, SLOT(setParamsButton()));
  connect(trajectories_list_, SIGNAL(activated(int)), this, SLOT(chooseTrajectoryParams(int)));
}

void ReferenceGeneratorPanel::trigger(bool checked) {
  p_active_ = checked;
  setParams();

  if (notifyParamsUpdate() && p_active_) {
    stop_button_->setEnabled(p_active_ && !p_stopped_);
    pause_button_->setEnabled(p_active_ && !p_paused_);
    play_button_->setEnabled(p_active_ && (p_stopped_ || p_paused_));
    set_button_->setEnabled(true);
    trajectories_list_->setEnabled(true);

    chooseTrajectoryParams(p_trajectory_type_);
  }
  else {
    stop_button_->setEnabled(false);
    pause_button_->setEnabled(false);
    play_button_->setEnabled(false);
    set_button_->setEnabled(false);
    trajectories_list_->setEnabled(false);

    chooseTrajectoryParams(-1);

    activate_checkbox_->setChecked(false);
    p_active_ = false;
    setParams();
  }
}

void ReferenceGeneratorPanel::setParamsButton() {
  setParams();
  notifyParamsUpdate();
}

void ReferenceGeneratorPanel::setParams() {
  verifyInputs();

  nh_local_.setParam("active", p_active_);
  nh_local_.setParam("trajectory_paused", p_paused_);
  nh_local_.setParam("trajectory_stopped", p_stopped_);

  nh_local_.setParam("trajectory_type", p_trajectory_type_);

  nh_local_.setParam("initial_x", p_x_0_);
  nh_local_.setParam("initial_y", p_y_0_);
  nh_local_.setParam("initial_theta", p_theta_0_);
  nh_local_.setParam("linear_velocity", p_v_);
  nh_local_.setParam("harmonic_period", p_T_);
  nh_local_.setParam("harmonic_radius_x", p_r_x_);
  nh_local_.setParam("harmonic_radius_y", p_r_y_);
  nh_local_.setParam("harmonic_multiplier_x", p_n_x_);
  nh_local_.setParam("harmonic_multiplier_y", p_n_y_);
}

void ReferenceGeneratorPanel::getParams() {
  nh_local_.param<bool>("active", p_active_, false);
  nh_local_.param<bool>("trajectory_paused", p_paused_, false);
  nh_local_.param<bool>("trajectory_stopped", p_stopped_, true);

  nh_local_.param<int>("trajectory_type", p_trajectory_type_, 0);

  nh_local_.param<double>("initial_x", p_x_0_, 0.0);
  nh_local_.param<double>("initial_y", p_y_0_, 0.0);
  nh_local_.param<double>("initial_theta", p_theta_0_, 0.0);
  nh_local_.param<double>("linear_velocity", p_v_, 0.0);
  nh_local_.param<double>("harmonic_period", p_T_, 0.0);
  nh_local_.param<double>("harmonic_radius_x", p_r_x_, 0.0);
  nh_local_.param<double>("harmonic_radius_y", p_r_y_, 0.0);
  nh_local_.param<double>("harmonic_multiplier_x", p_n_x_, 1.0);
  nh_local_.param<double>("harmonic_multiplier_y", p_n_y_, 1.0);
}

bool ReferenceGeneratorPanel::notifyParamsUpdate() {
  std_srvs::Empty empty;
  return params_cli_.call(empty);
}

bool ReferenceGeneratorPanel::verifyInputs() {
  p_trajectory_type_ = trajectories_list_->currentIndex();

  try { p_x_0_ = boost::lexical_cast<double>(x_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_x_0_ = 0.0; x_input_->setText("0.0"); return false; }

  try { p_y_0_ = boost::lexical_cast<double>(y_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_y_0_ = 0.0; y_input_->setText("0.0"); return false; }

  try { p_theta_0_ = boost::lexical_cast<double>(theta_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_theta_0_ = 0.0; theta_input_->setText("0.0"); return false; }

  try { p_v_ = boost::lexical_cast<double>(v_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_v_ = 0.0; v_input_->setText("0.0"); return false; }

  try { p_T_ = boost::lexical_cast<double>(T_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_T_ = 0.0; T_input_->setText("0.0"); return false; }

  try { p_r_x_ = boost::lexical_cast<double>(r_x_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_r_x_ = 0.0; r_x_input_->setText("0.0"); return false; }

  try { p_r_y_ = boost::lexical_cast<double>(r_y_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_r_y_ = 0.0; r_y_input_->setText("0.0"); return false; }

  try { p_n_x_ = boost::lexical_cast<double>(n_x_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_n_x_ = 0.0; n_x_input_->setText("0.0"); return false; }

  try { p_n_y_ = boost::lexical_cast<double>(n_y_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_n_y_ = 0.0; n_y_input_->setText("0.0"); return false; }

  return true;
}

void ReferenceGeneratorPanel::stop() {
  p_paused_ = false;
  p_stopped_ = true;

  stop_button_->setEnabled(false);
  pause_button_->setEnabled(true);
  play_button_->setEnabled(true);

  stop_button_->setChecked(true);
  pause_button_->setChecked(false);
  play_button_->setChecked(false);

  setParams();
  notifyParamsUpdate();
}

void ReferenceGeneratorPanel::pause() {
  p_paused_ = true;
  p_stopped_ = false;

  stop_button_->setEnabled(true);
  pause_button_->setEnabled(false);
  play_button_->setEnabled(true);

  stop_button_->setChecked(false);
  pause_button_->setChecked(true);
  play_button_->setChecked(false);

  setParams();
  notifyParamsUpdate();
}

void ReferenceGeneratorPanel::start() {
  p_paused_ = false;
  p_stopped_ = false;

  stop_button_->setEnabled(true);
  pause_button_->setEnabled(true);
  play_button_->setEnabled(false);

  stop_button_->setChecked(false);
  pause_button_->setChecked(false);
  play_button_->setChecked(true);

  setParams();
  notifyParamsUpdate();
}

void ReferenceGeneratorPanel::chooseTrajectoryParams(int index) {
  switch (index) {
  // Point
  case 0:
    x_input_->setEnabled(true);
    y_input_->setEnabled(true);
    theta_input_->setEnabled(true);
    v_input_->setEnabled(false);
    T_input_->setEnabled(false);
    r_x_input_->setEnabled(false);
    r_y_input_->setEnabled(false);
    n_x_input_->setEnabled(false);
    n_y_input_->setEnabled(false);
    break;

  // Linear
  case 1:
    x_input_->setEnabled(true);
    y_input_->setEnabled(true);
    theta_input_->setEnabled(true);
    v_input_->setEnabled(true);
    T_input_->setEnabled(false);
    r_x_input_->setEnabled(false);
    r_y_input_->setEnabled(false);
    n_x_input_->setEnabled(false);
    n_y_input_->setEnabled(false);
    break;

  // Harmonic and Lemniscate
  case 2:
  case 3:
    x_input_->setEnabled(true);
    y_input_->setEnabled(true);
    theta_input_->setEnabled(false);
    v_input_->setEnabled(false);
    T_input_->setEnabled(true);
    r_x_input_->setEnabled(true);
    r_y_input_->setEnabled(true);
    n_x_input_->setEnabled(true);
    n_y_input_->setEnabled(true);
    break;

  default:
    x_input_->setEnabled(false);
    y_input_->setEnabled(false);
    theta_input_->setEnabled(false);
    v_input_->setEnabled(false);
    T_input_->setEnabled(false);
    r_x_input_->setEnabled(false);
    r_y_input_->setEnabled(false);
    n_x_input_->setEnabled(false);
    n_y_input_->setEnabled(false);
    break;
  }
}

void ReferenceGeneratorPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ReferenceGeneratorPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

} // end namespace mrop_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrop_gui::ReferenceGeneratorPanel, rviz::Panel)
