// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>

#include "subsystems/Elevator.h"
#include "subsystems/Grabber.h"
#include "subsystems/Intake.h"
#include "Constants.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ElevatorRawDrive
    : public frc2::CommandHelper<frc2::CommandBase, ElevatorRawDrive> {
 public:
  ElevatorRawDrive(Elevator *elevator, Grabber *grabber, Intake *intake, frc::Joystick *ted);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Elevator *m_elevator;
  Grabber *m_grabber;
  Intake *m_intake;
  frc::Joystick *m_ted;
};
