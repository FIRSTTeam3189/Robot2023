// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Grabber.h"
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"

enum class ElevatorLevel { None, Low, Mid, High, DoubleSubstation };
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ElevatorPID
    : public frc2::CommandHelper<frc2::CommandBase, ElevatorPID> {
 public:
  ElevatorPID(Elevator *elevator, ElevatorLevel level, bool shouldFinish);
  ElevatorPID(Elevator *elevator, double level, bool shouldFinish);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Elevator *m_elevator;
  int m_withinThresholdLoops;
  double m_target;
  bool m_shouldFinish;
  ElevatorLevel m_level;
};
