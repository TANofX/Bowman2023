// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;

import javax.lang.model.type.UnionType;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
                        ///////////////////////  D R I V I N G ///////////////////////
    //BUTTON INPUTS FOR FLIGHT STICK CONTROLLER

    // Flight Stick Constants
    public static final int FLIGHTSTICK_PORT = 1;

    // Shooting
    public static final int FLIGHTSTICK_SHOOT_ALL_BUTTON = 1;
    public static final int FLIGHTSTICK_SHOOT_ONE_BUTTON = 2;
    public static final int FLIGHTSTICK_SHOOTER_STOP_BUTTON = 7;
    public static final int FLIGHTSTICK_LOW_SHOT_BUTTON = 3;
    public static final int FLIGHTSTICK_HIGH_GOAL_BUTTON = 5;
    public static final int FLIGHTSTICK_HIGH_GOAL_HARD_BUTTON = 4;
    public static final int FLIGHTSTICK_FULL_POWER_BUTTON = 6;

    // Climbing
    public static final int FLIGHTSTICK_HIGH_TO_TRAVERSAL = 8;
    public static final int XBOX_MID_TO_HIGH = 6;
    public static final int FLIGHTSTICK_GABE_PNEUMATICS_BUTTON = 9;
    public static final int FLIGHTSTICK_RACHEL_REACH_PNEUMATICS_BUTTON = 10;
    public static final int FLIGHTSTICK_LOCK_RACHEL_MOVE_JOYSTICK_BUTTON = 11;
    public static final int FLIGHTSTICK_CALIBRATE_CLIMBER_BUTTON = 12;


    public static final int MOVE_RACHEL_AXIS = 2;


 //   public static  int CLIMB_TRAVERSAL_BAR = 10;
 //   public static  int CLIMB_HIGH_BAR = 8;
  //  public static  int CANCEL_CLIMBER = 10;

                        ///////////////////////  C L I M B E R  ///////////////////////
    //  CLIMBER SUBSYSTEM CONSTANTS
    public static int RACHEL_UP_LEFT_LIMIT = 0;
    public static int RACHEL_UP_RIGHT_LIMIT = 1;
    public static int RACHEL_DOWN_LEFT_LIMIT = 2;
    public static int RACHEL_DOWN_RIGHT_LIMIT = 3;

    public static int GABE_CLAW = 1;
    public static int RACHEL_FORWARD_REACH = 3;
    public static int RACHEL_BACKWARD_REACH = 2;

    public static int GABE_LEFT_IDENTIFICATION = 3;
    public static int GABE_RIGHT_IDENTIFICATION = 2;
    public static int RACHEL_LEFT_BAR_SENSOR = 0;
    public static int RACHEL_RIGHT_BAR_SENSOR = 1;

    public static int RIGHT_RACHEL_FALCON = 12;
    public static int LEFT_RACHEL_FALCON = 13;

    public static double RACHEL_F = 0;
    public static double RACHEL_I = 0;
    public static double RACHEL_P = 0;

    public static double CLIMBER_THRESHOLD_CURRENT = 60;
    public static double CLIMBER_CURRENT_LIMIT = 39;
    public static double CLIMBER_THRESHOLD_TIMEOUT = 0.1;

    public static double CLIMB_HEIGHT_THRESHOLD = 400.0;
    public static double CLIMBER_MAX_VELOCITY = 9000.0;


    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .6223; // .6604; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = .5715; // .6985; // FIXME Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 2; // FIXME Set Pigeon ID

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7; // Falcon D                FIXME Set front left module drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; // Falcon E                FIXME Set front left module steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 13; // Encoder B          FIXME Set front left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(174.111); //  FIXME Measure and set front left steer offset

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 10; // Falcon G              FIXME Set front right drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11; // Falcon H              FIXME Set front right steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12; // Encoder A           FIXME Set front right steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(140.186); // FIXME Measure and set front right steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4; // Falcon A                FIXME Set back left drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5; // Falcon B                 FIXME Set back left steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 14; // Encoder C             FIXME Set back left steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(247.939); //   FIXME Measure and set back left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6; // Falcon C                FIXME Set back right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 9; // Falcon F                FIXME Set back right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 15; // Encoder D            FIXME Set back right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(244.951); //  FIXME Measure and set back right steer offset


    // Shooter Constants
    public static final int SHOOTER_CURRENT_LIMIT = 39;
    public static final int SHOOTER_THRESHOLD_CURRENT = 60;
    public static final double SHOOTER_THRESHOLD_TIMEOUT = 0.01;
    public static final double SHOOTER_F = 0.045; //0.04;
    public static final double SHOOTER_P = 0.125;
    public static final double SHOOTER_I = 0.0010;//0.00125;
    public static final double SHOOTER_SPEED = 0.9;
    public static final double SHOOTER_SPIN_ERROR = 80;
    public static final int PRIMARY_SHOOTER_ID = 16;
    public static final int SECONDARY_SHOOTER_ID = 17;

    // Cargo (Ball) Handler Constants
    public static final int HANDLERSENSOR_1_PORT = 0;
    public static final int HANDLERSENSOR_2_PORT = 1;
    public static final int HANDLERSENSOR_3_PORT = 2;
    public static final int HANDLERSENSOR_4_PORT = 3;
    public static final int TRANSIT_MOTOR_ID = 18;
    public static final double TRANSIT_MOTOR_SPEED = -0.5;
    public static final double CLEAR_TIMEOUT = 2.0;

    // Intake Constants
    public static final int INTAKE_SOLENOID_FORWARD = 1;
    public static final int INTAKE_SOLENOID_REVERSE = 0;

    public static final int DETECTABLE_DISTANCE = 250;
    public static final double REVERSE_INTAKE_TIMEOUT = 1.0;
    public static final int INTAKE_MOTOR_ID = 19;
    public static final double INTAKE_SPEED = 0.45;

    // Xbox Constants
    public static final int XBOX_PORT = 0;
    public static final int XBOX_RUNINTAKE_BUTTON = 3;
    public static final double XBOX_DEADBAND = 0.15;

    // Auto Drives PID
    public static final double DRIVE_POS_ERROR_CONTROLLER_P = 0.1;
    public static final double DRIVE_POS_ERROR_CONTROLLER_I = 0;
    public static final double DRIVE_POS_ERROR_CONTROLLER_D = 0;
    public static final double DRIVE_POS_ERROR_CONTROLLER_F = 0;
    
    public static final double DRIVE_AUTO_ROTATE_CONTROLLER_P = 2.4;
    public static final double DRIVE_AUTO_ROTATE_CONTROLLER_I = 0;
    public static final double DRIVE_AUTO_ROTATE_CONTROLLER_D = 0;

    public static final double DRIVE_MAX_ANGULAR_ACCEL = 600;
    public static final double DRIVE_MAX_ANGULAR_VELOCITY = 300;



    public static final class DriveConstants 
    {

        public static final int kLeftMotor1Port = 4;
        public static final int kLeftMotor2Port = 5;
        public static final int kRightMotor1Port = 6;
        public static final int kRightMotor2Port = 7;
    
        public static final int[] kLeftEncoderPorts = new int[] { 0, 1 };
        public static final int[] kRightEncoderPorts = new int[] { 2, 3 };
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;
    
        public static final double kTrackwidthMeters = 0.5842;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);
    
        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kGearBoxRatio = 1.0 / 12.727;
        // Need gear ratio for test bed
        // 12.727
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            ((kWheelDiameterMeters * Math.PI) * kGearBoxRatio) / (double) kEncoderCPR;
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        public static final double ksVolts = 0.58128;
        public static final double kvVoltSecondsPerMeter = 0.10789;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0069034;
    
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 0.29187;
        public static final double kMinimumTurnRate = 0.15;
        public static final double kThresholdCurrent = 40.0;
        public static final double kThresholdTimeout = 0.1;
        public static final double kCurrentLimit = 40.0;
      }
    
      public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
      }
    
      public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1.8;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.9;
    
        
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
      }    
        static class DriveTrainConstants {
            static final double kMaxSpeed = 3.0; // meters per second
            static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
            static final double kTrackWidth = 0.381 * 2; // meters
            static final double kWheelRadius = 0.0508; // meters
            static final int kEncoderResolution = 4096;
            static final double distancePerPulse = 2 * Math.PI * kWheelRadius / (double) kEncoderResolution;
        }
    
        static class FieldConstants {
            static final double length = Units.feetToMeters(47.527);
            static final double width = Units.feetToMeters(26.258);
            static final double gridDepth = Units.feetToMeters(4.504);
            static final double closeWidth = Units.feetToMeters(3.5);
            static final double closeMidWidth = Units.feetToMeters(9);
            static final double midWidth = Units.feetToMeters(14.5);
            static final double farWidth = Units.feetToMeters(22.129);
            static final double humanStationDepth = Units.feetToMeters(1.162);
        }
    
        static class VisionConstants {
            static final Transform3d robotToCam =
                    new Transform3d(
                            new Translation3d(0.5, 0.0, 0.5),
                            new Rotation3d(
                                    0, 0,
                                    0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
            // from center.
            static final String cameraName = "Global_Shutter_Camera";
        }
    }
    