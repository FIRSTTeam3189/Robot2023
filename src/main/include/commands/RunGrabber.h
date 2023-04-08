// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Grabber.h"


enum class GrabberAction { None, Grab, Shoot };
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RunGrabber
    : public frc2::CommandHelper<frc2::CommandBase, RunGrabber> {
 public:
  RunGrabber(Grabber *grabber, GrabberAction action);
  RunGrabber(Grabber *grabber, double power);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Grabber *m_grabber;
  double m_power;
  GrabberAction m_action;
};
