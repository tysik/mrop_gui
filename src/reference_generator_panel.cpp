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

namespace mtracker_gui
{

ReferenceGeneratorPanel::ReferenceGeneratorPanel(QWidget* parent) : rviz::Panel(parent), nh_("") {
  trigger_cli_ = nh_.serviceClient<mtracker::Trigger>("reference_generator_trigger_srv");
  params_cli_ = nh_.serviceClient<mtracker::Params>("reference_generator_params_srv");

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(false);

  stop_button_  = new QPushButton();
  pause_button_ = new QPushButton();
  play_button_  = new QPushButton();

  set_button_  = new QPushButton("Set");
  set_button_->setEnabled(false);

  stop_button_->setMinimumSize(50, 50);
  stop_button_->setMaximumSize(50, 50);
  stop_button_->setEnabled(false);
  stop_button_->setIcon(QIcon("/home/tysik/workspace/catkin_ws/src/mtracker_gui/resources/stop.png"));
  stop_button_->setIconSize(QSize(25, 25));

  pause_button_->setMinimumSize(50, 50);
  pause_button_->setMaximumSize(50, 50);
  pause_button_->setEnabled(false);
  pause_button_->setIcon(QIcon("/home/tysik/workspace/catkin_ws/src/mtracker_gui/resources/pause.png"));
  pause_button_->setIconSize(QSize(25, 25));

  play_button_->setMinimumSize(50, 50);
  play_button_->setMaximumSize(50, 50);
  play_button_->setEnabled(false);
  play_button_->setIcon(QIcon("/home/tysik/workspace/catkin_ws/src/mtracker_gui/resources/play.png"));
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
  trajectories_list_->setEnabled(false);

  x_input_ = new QLineEdit("0.0");
  y_input_ = new QLineEdit("0.0");
  theta_input_ = new QLineEdit("0.0");
  v_input_ = new QLineEdit("0.1");
  T_input_ = new QLineEdit("5.0");
  Rx_input_ = new QLineEdit("1.0");
  Ry_input_ = new QLineEdit("1.0");
  nx_input_ = new QLineEdit("1.0");
  ny_input_ = new QLineEdit("2.0");

  x_input_->setEnabled(false);
  y_input_->setEnabled(false);
  theta_input_->setEnabled(false);
  v_input_->setEnabled(false);
  T_input_->setEnabled(false);
  Rx_input_->setEnabled(false);
  Ry_input_->setEnabled(false);
  nx_input_->setEnabled(false);
  ny_input_->setEnabled(false);

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
  inputs_layout->addWidget(nx_input_, 1, 5);
  inputs_layout->addWidget(new QLabel("-, "), 1, 6);
  inputs_layout->addWidget(new QLabel("n<sub>y</sub>:"), 1, 7);
  inputs_layout->addWidget(ny_input_, 1, 8);
  inputs_layout->addWidget(new QLabel("-"), 1, 9);
  inputs_layout->addItem(margin, 1, 10);
  //
  inputs_layout->addItem(margin, 2, 0);
  inputs_layout->addWidget(new QLabel("T:"), 2, 1);
  inputs_layout->addWidget(T_input_, 2, 2);
  inputs_layout->addWidget(new QLabel("s, "), 2, 3);
  inputs_layout->addWidget(new QLabel("R<sub>x</sub>:"), 2, 4);
  inputs_layout->addWidget(Rx_input_, 2, 5);
  inputs_layout->addWidget(new QLabel("m, "), 2, 6);
  inputs_layout->addWidget(new QLabel("R<sub>y</sub>:"), 2, 7);
  inputs_layout->addWidget(Ry_input_, 2, 8);
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
  connect(set_button_, SIGNAL(clicked()), this, SLOT(updateParams()));
  connect(trajectories_list_, SIGNAL(activated(QString)), this, SLOT(chooseTrajectory(QString)));
}

void ReferenceGeneratorPanel::trigger(bool checked) {
  mtracker::Trigger trigger;
  trigger.request.activate = checked;

  if (trigger_cli_.call(trigger)) {
    if (checked) {
      stop_button_->setEnabled(true);
      pause_button_->setEnabled(true);
      play_button_->setEnabled(true);
      set_button_->setEnabled(true);
      trajectories_list_->setEnabled(true);

      QString traj_type = trajectories_list_->currentText();
      chooseTrajectory(traj_type);
    }
    else {
      stop_button_->setEnabled(false);
      pause_button_->setEnabled(false);
      play_button_->setEnabled(false);
      set_button_->setEnabled(false);
      trajectories_list_->setEnabled(false);

      x_input_->setEnabled(false);
      y_input_->setEnabled(false);
      theta_input_->setEnabled(false);
      v_input_->setEnabled(false);
      T_input_->setEnabled(false);
      Rx_input_->setEnabled(false);
      Ry_input_->setEnabled(false);
      nx_input_->setEnabled(false);
      ny_input_->setEnabled(false);
    }
  }
  else {
    activate_checkbox_->setChecked(!checked);
  }
}

void ReferenceGeneratorPanel::updateParams() {
  mtracker::Params params;
  params.request.params.resize(13, 0);

  stop_button_->setEnabled(false);
  pause_button_->setEnabled(true);
  play_button_->setEnabled(true);

  params.request.params[2] = 1.0;   // Update traj. params

  QString traj_type = trajectories_list_->currentText();
  if (traj_type == "Point")
    params.request.params[3] = 0.0;
  else if (traj_type == "Linear")
    params.request.params[3] = 1.0;
  else if (traj_type == "Harmonic")
    params.request.params[3] = 2.0;
  else if (traj_type == "Lemniscate")
    params.request.params[3] = 3.0;

  try {params.request.params[4] = boost::lexical_cast<double>(x_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &){ x_input_->setText(":-("); return; }

  try {params.request.params[5] = boost::lexical_cast<double>(y_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &){ y_input_->setText(":-("); return; }

  try {params.request.params[6] = boost::lexical_cast<double>(theta_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &){ theta_input_->setText(":-("); return; }

  try {params.request.params[7] = boost::lexical_cast<double>(v_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &){ v_input_->setText(":-("); return; }

  try {params.request.params[8] = boost::lexical_cast<double>(T_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &){ T_input_->setText(":-("); return; }

  try {params.request.params[9] = boost::lexical_cast<double>(Rx_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &){ Rx_input_->setText(":-("); return; }

  try {params.request.params[10] = boost::lexical_cast<double>(Ry_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &){ Ry_input_->setText(":-("); return; }

  try {params.request.params[11] = boost::lexical_cast<double>(nx_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &){ nx_input_->setText(":-("); return; }

  try {params.request.params[12] = boost::lexical_cast<double>(ny_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &){ ny_input_->setText(":-("); return; }

  params_cli_.call(params);
}

void ReferenceGeneratorPanel::stop() {
  mtracker::Params params;
  params.request.params.resize(13, 0);
  params.request.params[0] = 0.0; // start = false
  params.request.params[1] = 0.0; // pause = false

  if (params_cli_.call(params)) {
    stop_button_->setEnabled(false);
    pause_button_->setEnabled(true);
    play_button_->setEnabled(true);
  }
}

void ReferenceGeneratorPanel::pause() {
  mtracker::Params params;
  params.request.params.resize(13, 0.0);
  params.request.params[0] = 0.0;
  params.request.params[1] = 1.0;

  if (params_cli_.call(params)) {
    stop_button_->setEnabled(true);
    pause_button_->setEnabled(false);
    play_button_->setEnabled(true);
  }
}

void ReferenceGeneratorPanel::start() {
  mtracker::Params params;
  params.request.params.resize(13, 0.0);
  params.request.params[0] = 1.0;
  params.request.params[1] = 0.0;


  if (params_cli_.call(params)) {
    stop_button_->setEnabled(true);
    pause_button_->setEnabled(true);
    play_button_->setEnabled(false);
  }
}

void ReferenceGeneratorPanel::chooseTrajectory(QString traj_type) {
  if (traj_type == "Point") {
    x_input_->setEnabled(true);
    y_input_->setEnabled(true);
    theta_input_->setEnabled(true);
    v_input_->setEnabled(false);
    T_input_->setEnabled(false);
    Rx_input_->setEnabled(false);
    Ry_input_->setEnabled(false);
    nx_input_->setEnabled(false);
    ny_input_->setEnabled(false);
  }
  else if (traj_type == "Linear") {
    x_input_->setEnabled(true);
    y_input_->setEnabled(true);
    theta_input_->setEnabled(true);
    v_input_->setEnabled(true);
    T_input_->setEnabled(false);
    Rx_input_->setEnabled(false);
    Ry_input_->setEnabled(false);
    nx_input_->setEnabled(false);
    ny_input_->setEnabled(false);
  }
  else if (traj_type == "Harmonic" || traj_type == "Lemniscate") {
    x_input_->setEnabled(true);
    y_input_->setEnabled(true);
    theta_input_->setEnabled(false);
    v_input_->setEnabled(false);
    T_input_->setEnabled(true);
    Rx_input_->setEnabled(true);
    Ry_input_->setEnabled(true);
    nx_input_->setEnabled(true);
    ny_input_->setEnabled(true);
  }
}

void ReferenceGeneratorPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ReferenceGeneratorPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::ReferenceGeneratorPanel, rviz::Panel)
