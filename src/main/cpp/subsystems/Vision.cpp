// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"

Vision::Vision() : m_Data() {
    auto nts = nt::NetworkTableInstance::GetDefault();
    m_DetectionTypeTopic = nts.GetIntegerTopic("Vision/Detection");
    m_IDTopic = nts.GetIntegerTopic("Vision/AprilTag/ID");
    m_TranslationMatrixTopic = nts.GetFloatArrayTopic("Vision/AprilTag/TMatrix");
    m_cosZRotTopic = nts.GetFloatTopic("Vision/AprilTag/RMatrix");
    auto enableTopic = nts.GetBooleanTopic("Enable");
    m_EnablePub = enableTopic.Publish();
    m_EnablePub.Set(m_EnabledState);

    // std::cout << "Vision constructed\n";
}

// This method will be called once per scheduler run
void Vision::Periodic() {
    // std::cout << "Vision periodic\n";
    if (m_EnabledState) {
        // std::cout << "Vision enabled\n";
        int detection = m_DetectionTypeTopic.Subscribe(-1).Get();
        // std::cout << "Detection type: " << detection << "\n";
        if (detection == 0) {
            m_Data.detectionID = DetectionType::None;
        }
        else if (detection == 1) {
            // std::cout << "AprilTag detected\n";
            m_Data.detectionID = DetectionType::AprilTag;
            m_Data.ID = m_IDTopic.Subscribe(-1).Get();
            // std::cout << "Data ID " << m_Data.ID << "\n";
            float a[]{0.0f, 0.0f, 0.0f};
            std::span s{a, std::size(a)};

            m_Data.translationMatrix = m_TranslationMatrixTopic.Subscribe(s).Get();
            // Round values to nearest inch so they don't fluctuate too much
            // m_Data.translationMatrix[0] = Round((double)m_Data.translationMatrix[0], 0.025);
            // m_Data.translationMatrix[1] = Round((double)m_Data.translationMatrix[1], 0.025);
            m_Data.translationMatrix[0] += CAMERA_X_OFFSET;
            m_Data.translationMatrix[1] += CAMERA_Y_OFFSET;
            // std::cout << "X and Y translations " << m_Data.translationMatrix[0] << " " << m_Data.translationMatrix[1] << "\n";
            m_Data.cosZRot = m_cosZRotTopic.Subscribe(0.0f).Get();
            // m_Data.cosZRot = Round((double)m_Data.cosZRot, 0.005);
            // std::cout << "cosZRot " << m_Data.cosZRot << "\n";
        }
        else if (detection == 2) {
            m_Data.detectionID = DetectionType::Contours;
        }
        else {
            // std::cout << "Invalid detection type\n";
        }
    }
}

double Vision::Round(double value, double roundAmount) {
    double upperBound = value + roundAmount;
    double lowerBound = value - roundAmount;

    double roundedValue = 0.0;
    if (abs(upperBound - value) <= abs(value - lowerBound)) {
        roundedValue = upperBound;
    }
    else {
        roundedValue = lowerBound;
    }

    return roundedValue;
}

void Vision::Toggle() {
    m_EnabledState = !m_EnabledState;
    m_EnablePub.Set(m_EnabledState);
}

VisionData Vision::GetData() {
    return m_Data;
} 