// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GreekArm extends SubsystemBase {
  private static final double HERCULES_LENGTH = 38.586;
  private static final double ARTEMIS_LENGTH = 27.019;
  private static final double APOLLO_LENGTH = 31.5;

  /** Creates a new GreekArm. */
  public GreekArm() {}

  public static Translation3d calculateKinematics(double shoulderThetaDegrees, double elbowThetaDegrees) {
    double sTR = shoulderThetaDegrees * Math.PI / 180.0;
    double eTR = elbowThetaDegrees * Math.PI / 180.0;
    double tTR = sTR + eTR;

    double x = APOLLO_LENGTH * Math.cos(tTR) + ARTEMIS_LENGTH * Math.cos(sTR);
    double z = HERCULES_LENGTH + APOLLO_LENGTH * Math.sin(tTR) + ARTEMIS_LENGTH * Math.sin(sTR);

    return new Translation3d(x, 0.0, z);
  }

  public static List<Rotation2d> calculateInverseKinematics(double x, double z) {
    ArrayList<Rotation2d> returnList = new ArrayList<Rotation2d>(2);
    double zPrime = z - HERCULES_LENGTH;

    double elbow = Math.acos((Math.pow(x, 2.0) + Math.pow(zPrime, 2.0) - (Math.pow(ARTEMIS_LENGTH, 2.0) + Math.pow(APOLLO_LENGTH, 2.0))) / (2.0 * APOLLO_LENGTH * ARTEMIS_LENGTH));
    double shoulder = Math.atan2(zPrime, x) - Math.atan2(APOLLO_LENGTH * Math.sin(elbow), ARTEMIS_LENGTH + APOLLO_LENGTH * Math.cos(elbow));

    returnList.add(new Rotation2d(shoulder));
    returnList.add(new Rotation2d(elbow));

    elbow = -1 * Math.acos((Math.pow(x, 2.0) + Math.pow(zPrime, 2.0) - (Math.pow(ARTEMIS_LENGTH, 2.0) + Math.pow(APOLLO_LENGTH, 2.0))) / (2.0 * APOLLO_LENGTH * ARTEMIS_LENGTH));
    shoulder = Math.atan2(zPrime, x) - Math.atan2(APOLLO_LENGTH * Math.sin(elbow), ARTEMIS_LENGTH + APOLLO_LENGTH * Math.cos(elbow));

    returnList.add(new Rotation2d(shoulder));
    returnList.add(new Rotation2d(elbow));

    return returnList;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
