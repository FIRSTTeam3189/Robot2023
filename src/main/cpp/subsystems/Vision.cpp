// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"

Vision::Vision() : m_Data() {
    // Read the vision data from the coprocessor
    auto nts = nt::NetworkTableInstance::GetDefault();
    m_DetectionTypeTopic = nts.GetIntegerTopic("Vision/Detection");
    m_IDTopic = nts.GetIntegerTopic("Vision/AprilTag/ID");
    m_TranslationMatrixTopic = nts.GetFloatArrayTopic("Vision/AprilTag/TMatrix");
    m_cosZRotTopic = nts.GetFloatTopic("Vision/AprilTag/RMatrix");
    auto enableTopic = nts.GetBooleanTopic("Enable");
    m_EnablePub = enableTopic.Publish();
    m_EnablePub.Set(m_EnabledState);
}

// This method will be called once per scheduler run
void Vision::Periodic() {
    // If vision is enabled and sending data, checks what the vision detects
    // Allows for different behavior based on tape and apriltags
    if (m_EnabledState) {
        int detection = m_DetectionTypeTopic.Subscribe(-1).Get();
        if (detection == 0) {
            m_Data.detectionID = DetectionType::None;
        }
        else if (detection == 1) {
            m_Data.detectionID = DetectionType::AprilTag;
            // Get ID of AprilTag -- corresponds to known position on field
            // Can be used to update robot's odometry
            m_Data.ID = m_IDTopic.Subscribe(-1).Get();
            float a[]{0.0f, 0.0f, 0.0f};
            std::span s{a, std::size(a)};

            // Get x, y, z distances from camera to target and offsets x and y
            // This translates the camera as if it were at the front and center of the robot's frame
            // Which is what the vision alignment should be based off of
            m_Data.translationMatrix = m_TranslationMatrixTopic.Subscribe(s).Get();
            m_Data.translationMatrix[0] += CAMERA_X_OFFSET;
            m_Data.translationMatrix[1] += CAMERA_Y_OFFSET;
            m_Data.cosZRot = m_cosZRotTopic.Subscribe(0.0f).Get();
        }
        else if (detection == 2) {
            m_Data.detectionID = DetectionType::Contours;
        }
        else {
            
        }
    }
}

void Vision::Toggle() {
    m_EnabledState = !m_EnabledState;
    m_EnablePub.Set(m_EnabledState);
}

VisionData Vision::GetData() {
    return m_Data;
} 