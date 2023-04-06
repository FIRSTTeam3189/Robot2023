// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#undef max

#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <networktables/FloatArrayTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/FloatTopic.h>

#include "Constants.h"
#include <iostream>
#include <span>

enum class DetectionType {None, AprilTag, Contours};

struct VisionData {
  int ID;
  std::vector<float> translationMatrix{0.0f, 0.0f, 0.0f};
  float cosZRot;
  DetectionType detectionID;
};

class Vision : public frc2::SubsystemBase {
public:
  Vision();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  VisionData GetData();
  void Toggle();

private:
  nt::IntegerTopic m_IDTopic;
  nt::IntegerTopic m_DetectionTypeTopic;
  nt::FloatArrayTopic m_TranslationMatrixTopic;
  nt::FloatTopic m_cosZRotTopic;
  nt::BooleanPublisher m_EnablePub;
  VisionData m_Data;
  bool m_EnabledState = true;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
