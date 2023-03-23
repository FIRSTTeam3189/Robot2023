// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Elevator.h"
#include "subsystems/Grabber.h"
#include "subsystems/Intake.h"

#include "commands/ElevatorPID.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class UltraShoot
    : public frc2::CommandHelper<frc2::CommandBase, UltraShoot> {
 public:
  UltraShoot(Elevator *elevator, Intake *intake, Grabber *grabber);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Elevator *m_elevator;
  Intake *m_intake;
  Grabber *m_grabber;
};
