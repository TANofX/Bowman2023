// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public enum ArmPositions {
    HOME(250, -160),
    SAFE_TRANSITION(172.5, -90),
    PRE_PRE_PICKUP(172.5, 0),
    PRE_PICKUP(90, 140),
    PICK_UP(190, 115), 
    PRE_SCORING(100, 100),
    MID_SCORE(118.5, 97),
    LEAVE_SCORING(150, -10),
    HIGH_SCORE(150, 20),
    UNKNOWN(251.5, -164.3);
    private static final double POSITION_TOLERANCE = 15;
    public Rotation2d shoulderRotation;
    public Rotation2d elbowRotation;
    ArmPositions(double shoulderAngle, double elbowAngle) {
        shoulderRotation = Rotation2d.fromDegrees(shoulderAngle);
        elbowRotation = Rotation2d.fromDegrees(elbowAngle);
    }
    public static ArmPositions getPosition(Rotation2d shoulderAngle, Rotation2d elbowAngle) {
        for (ArmPositions current: values()) {
            if (current == UNKNOWN) continue;
            if ((Math.abs(shoulderAngle.getDegrees() - current.shoulderRotation.getDegrees()) < POSITION_TOLERANCE) && (Math.abs(elbowAngle.getDegrees() - current.elbowRotation.getDegrees()) < POSITION_TOLERANCE) ) {
                return current;
            }
        }
        return UNKNOWN;
    }
}
