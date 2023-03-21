// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PhotonCameraWrapper;

import static frc.robot.Constants.*;

import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double 
  MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
//  public static final double MAX_VELOCITY_METERS_PER_SECOND = 1500.0 / 60.0 *
          SdsModuleConfigurations.MK4_L1.getDriveReduction() *
          SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Remove if you are using a Pigeon
 // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  public final static Pigeon2 m_pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);
  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  private final SwerveModule swerveModules[];

  private GenericEntry driveEntry;
  private GenericEntry rotationEntry;

  private final Field2d fieldSim = new Field2d();
  public PhotonCameraWrapper pcw;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private SwerveDrivePoseEstimator swervePoseEstimator; 
  private boolean ignoreAprilTags = true;
  
  public DrivetrainSubsystem() {
        m_pigeon.setYaw(0.0);
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        tab.addNumber("Pigeon", ()->{return m_pigeon.getYaw();});
        driveEntry = tab.add("driveFraction", 1)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", 0.1, "max", 1))
                        .getEntry();
        rotationEntry = tab.add("rotationFraction", 1)
                           .withWidget(BuiltInWidgets.kNumberSlider)
                           .withProperties(Map.of("min", 0.1, "max", 1))
                           .getEntry();


        

        pcw = new PhotonCameraWrapper();
       

        tab.addNumber("Reported Yaw", () -> {return getGyroscopeRotation().getDegrees();});
        tab.addNumber("max velocity", () -> {return MAX_VELOCITY_METERS_PER_SECOND;} );

        m_frontLeftModule = new MkSwerveModuleBuilder (MkModuleConfiguration.getDefaultSteerFalcon500())
                .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
                        .withGearRatio(SdsModuleConfigurations.MK4_L1)
                        .withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR)
                        .withSteerMotor(MotorType.FALCON, FRONT_LEFT_MODULE_STEER_MOTOR)
                        .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
                        .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
                        .build();

        m_frontRightModule = new MkSwerveModuleBuilder (MkModuleConfiguration.getDefaultSteerFalcon500())
                .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
                        .withGearRatio(SdsModuleConfigurations.MK4_L1)
                        .withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
                        .withSteerMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_STEER_MOTOR)
                        .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
                        .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
                        .build();            

        m_backLeftModule = new MkSwerveModuleBuilder (MkModuleConfiguration.getDefaultSteerFalcon500())
                .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
                        .withGearRatio(SdsModuleConfigurations.MK4_L1)
                        .withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR)
                        .withSteerMotor(MotorType.FALCON, BACK_LEFT_MODULE_STEER_MOTOR)
                        .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
                        .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
                        .build();     

         m_backRightModule = new MkSwerveModuleBuilder (MkModuleConfiguration.getDefaultSteerFalcon500())
                .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
                        .withGearRatio(SdsModuleConfigurations.MK4_L1)
                        .withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR)
                        .withSteerMotor(MotorType.FALCON, BACK_RIGHT_MODULE_STEER_MOTOR)
                        .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
                        .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
                        .build();     

        tab.addNumber("BL Absolute", ()->{return ((WPI_CANCoder)m_backLeftModule.getSteerEncoder().getInternal()).getAbsolutePosition();});
        tab.addNumber("FL Absolute", ()->{return ((WPI_CANCoder)m_frontLeftModule.getSteerEncoder().getInternal()).getAbsolutePosition();});
        tab.addNumber("BR Absolute", ()->{return ((WPI_CANCoder)m_backRightModule.getSteerEncoder().getInternal()).getAbsolutePosition();});
        tab.addNumber("FR Absolute", ()->{return ((WPI_CANCoder)m_frontRightModule.getSteerEncoder().getInternal()).getAbsolutePosition();});
//     m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
//             // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
//             tab.getLayout("Front Left Module", BuiltInLayouts.kList)
//                     .withSize(2, 4)
//                     .withPosition(0, 0),
//             // This can either be STANDARD or FAST depending on your gear configuration
//             Mk3SwerveModuleHelper.GearRatio.STANDARD,
//             // This is the ID of the drive motor
//             FRONT_LEFT_MODULE_DRIVE_MOTOR,
//             // This is the ID of the steer motor
//             FRONT_LEFT_MODULE_STEER_MOTOR,
//             // This is the ID of the steer encoder
//             FRONT_LEFT_MODULE_STEER_ENCODER,
//             // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
//             FRONT_LEFT_MODULE_STEER_OFFSET
//     );

    // We will do the same for the other modules
//     m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
//             tab.getLayout("Front Right Module", BuiltInLayouts.kList)
//                     .withSize(2, 4)
//                     .withPosition(2, 0),
//             Mk3SwerveModuleHelper.GearRatio.STANDARD,
//             FRONT_RIGHT_MODULE_DRIVE_MOTOR,
//             FRONT_RIGHT_MODULE_STEER_MOTOR,
//             FRONT_RIGHT_MODULE_STEER_ENCODER,
//             FRONT_RIGHT_MODULE_STEER_OFFSET
//     );

//     m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
//             tab.getLayout("Back Left Module", BuiltInLayouts.kList)
//                     .withSize(2, 4)
//                     .withPosition(4, 0),
//             Mk3SwerveModuleHelper.GearRatio.STANDARD,
//             BACK_LEFT_MODULE_DRIVE_MOTOR,
//             BACK_LEFT_MODULE_STEER_MOTOR,
//             BACK_LEFT_MODULE_STEER_ENCODER,
//             BACK_LEFT_MODULE_STEER_OFFSET
//     );

//     m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
//             tab.getLayout("Back Right Module", BuiltInLayouts.kList)
//                     .withSize(2, 4)
//                     .withPosition(6, 0),
//             Mk3SwerveModuleHelper.GearRatio.STANDARD,
//             BACK_RIGHT_MODULE_DRIVE_MOTOR,
//             BACK_RIGHT_MODULE_STEER_MOTOR,
//             BACK_RIGHT_MODULE_STEER_ENCODER,
//             BACK_RIGHT_MODULE_STEER_OFFSET
//     );

        swerveModules = new SwerveModule[4];
        swerveModules[0] = m_frontLeftModule;
        swerveModules[1] = m_frontRightModule;
        swerveModules[2] = m_backLeftModule;
        swerveModules[3] = m_backRightModule;

        swervePoseEstimator = new SwerveDrivePoseEstimator(m_kinematics, getAngleRotation2d(), getModulePositions(), new Pose2d());
        ShuffleboardTab estTab = Shuffleboard.getTab("Estimated Robot Position");
        estTab.addNumber("X", () -> {return swervePoseEstimator.getEstimatedPosition().getX();});
        estTab.addNumber("Y", () -> {return swervePoseEstimator.getEstimatedPosition().getY();});
        estTab.addNumber("Heading", () -> {return swervePoseEstimator.getEstimatedPosition().getRotation().getDegrees();});

        estTab.addBoolean("Targets Visisble", () -> {return (pcw.getTargets().getTargets().size() > 0);});
        estTab.add(fieldSim);

        tab.addNumber("Front Left Speed", ()->{return swerveModules[0].getDriveVelocity();});
        tab.addNumber("Front Right Speed", ()->{return swerveModules[1].getDriveVelocity();});
        tab.addNumber("Back Left Speed", ()->{return swerveModules[2].getDriveVelocity();});
        tab.addNumber("Back Right Speed", ()->{return swerveModules[3].getDriveVelocity();});
}

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    // FIXME Remove if you are using a Pigeon
    //m_pigeon.setFusedHeading(0.0);    Changed for pigeon 2  Idk What I am doing
    m_pigeon.setYaw(0.0);
  }

  public Rotation2d getGyroscopeRotation() {
    // FIXME Remove if you are using a Pigeon
  //  return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());        Changed for pigeon 2 - idk what I am doing
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;

    defaultDrive();
  }

  @Override
  public void periodic() {
    //odometry.update(getAngleRotation2d(), getModulePositions());
    updateOdometry();
  }

  public void defaultDrive() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveModuleState[] currentStates = getModuleStates();
                 for(int i=0;i<4;i++){
              states[i]=SwerveModuleState.optimize(states[i], currentStates[i].angle);
    
              if (states[i].speedMetersPerSecond == 0) {
                    states[i].angle = currentStates[i].angle;
              }
             } 
         SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    
         setModuleStates(states);    
  }

  public void updateOdometry() {
        swervePoseEstimator.update(getGyroscopeRotation(), getModulePositions());

        // Also apply vision measurements. We use 0.3 seconds in the past as an example
        // -- o
        // a real robot, this must be calculated based either on latency or timestamps.
        Optional<EstimatedRobotPose> result = pcw.getEstimatedGlobalPose(swervePoseEstimator.getEstimatedPosition());

        if (result.isPresent() && !ignoreAprilTags) {
                EstimatedRobotPose camPose = result.get();
                swervePoseEstimator.addVisionMeasurement(
                camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
                fieldSim.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
        } else {
                // move it way off the screen to make it disappear
                fieldSim.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
        }

        fieldSim.setRobotPose(swervePoseEstimator.getEstimatedPosition());
}
public void useAprilTags(boolean b) {
        ignoreAprilTags = !b;
}

private void setModuleStates(SwerveModuleState[] states) {
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());    
}

private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                new SwerveModuleState(swerveModules[0].getDriveVelocity(), new Rotation2d(swerveModules[0].getSteerAngle())),
                new SwerveModuleState(swerveModules[1].getDriveVelocity(), new Rotation2d(swerveModules[1].getSteerAngle())),
                new SwerveModuleState(swerveModules[2].getDriveVelocity(), new Rotation2d(swerveModules[2].getSteerAngle())),
                new SwerveModuleState(swerveModules[3].getDriveVelocity(), new Rotation2d(swerveModules[3].getSteerAngle())),
        };
}

private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
        };
}

public Pose2d getPoseMeters() {
        return swervePoseEstimator.getEstimatedPosition(); //odometry.getPoseMeters();
}

public void drive(int i, int j, int k, boolean b) {
}

public void resetOdometry(Pose2d pose2d) {
        swervePoseEstimator.resetPosition(getAngleRotation2d(), getModulePositions(), pose2d);
        //resetGyroscope(pose2d.getRotation().getDegrees());
}

public void enableBrakeMode(boolean b) {
        for (SwerveModule m: swerveModules) {
                WPI_TalonFX currentMotor = (WPI_TalonFX)m.getDriveMotor();

                if (b) {
                        currentMotor.setNeutralMode(NeutralMode.Brake);
                }
                else {
                        currentMotor.setNeutralMode(NeutralMode.Coast);
                }
        }
}

public Rotation2d getAngleRotation2d() {
    return getGyroscopeRotation();
}

public double getRoll() {
        return m_pigeon.getRoll();
}

public double getPitch() {
        return m_pigeon.getPitch();
}

public void resetGyroscope(double d) {
        m_pigeon.setYaw(d);
}

// Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
             (Command)new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                   this.resetOdometry(traj.getInitialHolonomicPose());
               }
             }),
             (Command)new PPSwerveControllerCommand(
                 traj, 
                 this::getPoseMeters, // Pose supplier
                 this.m_kinematics, // SwerveDriveKinematics
                 new PIDController(Constants.DRIVE_POS_ERROR_CONTROLLER_P, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 new PIDController(Constants.DRIVE_POS_ERROR_CONTROLLER_P, 0, 0), // Y controller (usually the same values as X controller)
                 new PIDController(Constants.DRIVE_AUTO_ROTATE_CONTROLLER_P, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 this::setModuleStates, // Module states consumer
                 false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                 this // Requires this drive subsystem
             )
         );
     }

public double getdriveFraction() {
        return driveEntry.getDouble(1.0);
}

public double getrotationFraction() {
    return rotationEntry.getDouble(1.0);
}

}