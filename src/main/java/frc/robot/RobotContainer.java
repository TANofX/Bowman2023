// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autobalance;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.LowerIntake;
import frc.robot.commands.ManualArm;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.RaiseIntake;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GreekArm;
import frc.robot.commands.RunConveyer;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ZeroYaw;
import frc.robot.commands.Autobalance.BalancePoint;
import frc.robot.subsystems.ConveyerBelt;
import frc.robot.subsystems.FlapperIntake;

import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public static final FlapperIntake m_intake = new FlapperIntake();
  public static final ConveyerBelt m_conveyer = new ConveyerBelt();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.XBOX_PORT);
  public static final GreekArm m_arm = new GreekArm();
  private final CommandXboxController m_operatorController = 
      new CommandXboxController(Constants.XBOX_PORT_2);

private SendableChooser<Command> autChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureAutos();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));
    // m_operatorController.povUp().whileTrue(new InstantCommand(() -> m_arm.moveshoulderup()));
    // m_operatorController.povDown().whileTrue(new InstantCommand(() -> m_arm.moveshoulderdown()));
    // m_operatorController.povLeft().whileTrue(new InstantCommand(() -> m_arm.moveelbowup()));
    // m_operatorController.povRight().whileTrue(new InstantCommand(() -> m_arm.elbowdown()));

    m_operatorController.y().onTrue(new InstantCommand(() -> m_arm.stopArm()));

    // m_driverController.a().onTrue(new MoveArmToPosition(180, -85).andThen(new MoveArmToPosition(142, 39.4)));
    // m_driverController.b().onTrue(new MoveArmToPosition(180, -85).andThen(new MoveArmToPosition(254, -164)));
    // m_driverController.x().onTrue(new InstantCommand(() -> m_arm.stopArm() ));


    // m_driverController.povDown().onTrue(new MoveArmToPosition(150, -80).andThen(new MoveArmToPosition(189, 118)));

    // m_driverController.povUp().onTrue(new MoveArmToPosition(180, -85).andThen(new MoveArmToPosition(122.3, 91.1)));

    // m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_arm.setEndEffectorSpeeds(-0.2, 0)));
    m_operatorController.button(7).onTrue(new InstantCommand(() -> m_arm.toggleGripper()));
    // m_driverController.y().onTrue(new Autobalance(Autobalance.BalancePoint.FORWARD));
    // m_driverController.start().onTrue(new DriveFollowPath("P1 2 (place, out, take, back, place)", 2.0, 0.5, true));
    // // m_driverController.b().onTrue(new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(m_drivetrainSubsystem.getPoseMeters())));    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,

    // m_driverController.a().onTrue(new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()));
    // m_driverController.b().onTrue(new DriveFollowPath("Around The Charge", 2.0, 0.5, true));//Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, true));
    // m_driverController.x().onTrue(new Autobalance(Autobalance.BalancePoint.LEVEL));//Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, true));
    // m_driverController.y().onTrue(new Autobalance(Autobalance.BalancePoint.FORWARD));
    // m_driverController.start().onTrue(new DriveFollowPath("P1 2 (place, out, take, back, place)", 2.0, 0.5, true));

    m_driverController.rightTrigger().whileTrue(new RunIntake(.9));
    m_driverController.leftTrigger().whileTrue(new RunIntake(.25));

    m_driverController.leftBumper().onTrue(new RaiseIntake());
    m_driverController.rightBumper().onTrue(new LowerIntake());

    m_driverController.povUp().onTrue(new Autobalance(Autobalance.BalancePoint.FORWARD));
    m_driverController.povDown().onTrue(new Autobalance(Autobalance.BalancePoint.BACKWARD));
    m_driverController.a().onTrue(new Autobalance(Autobalance.BalancePoint.LEVEL));



    m_conveyer.setDefaultCommand(new RunConveyer(() -> {return modifyAxis(m_operatorController.getRightY());}));
    
  

    //m_driverController.povUp().onTrue(new InstantCommand(() -> {m_intake.toggleIntakePosition();}));
    m_operatorController.leftBumper().onTrue(new InstantCommand(() -> {if (m_conveyer.getState() == DoubleSolenoid.Value.kReverse) {m_conveyer.openConveyer();} else {m_conveyer.closeConveyer();}}));
    
    // m_driverController.b().onTrue(new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(m_drivetrainSubsystem.getPoseMeters())));    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    RobotContainer.m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            new DoubleSupplier() {
              @Override
                  public double getAsDouble() {
                      return -modifyAxis(m_driverController.getLeftY()) * (m_driverController.getLeftTriggerAxis() + 1) / 2.0 * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
                  }
            },
            new DoubleSupplier() {
              @Override
              public double getAsDouble() {
                  return -modifyAxis(m_driverController.getLeftX()) * (m_driverController.getLeftTriggerAxis() + 1) / 2.0  * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
              }
            },
            new DoubleSupplier() {
              public double getAsDouble() {
                return -modifyAxis(m_driverController.getRightX()) * (m_driverController.getLeftTriggerAxis() + 1) / 2.0  * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
              }
            }));

            m_arm.setDefaultCommand(new ManualArm(
              new DoubleSupplier() {
                @Override
                public double getAsDouble() {return modifyAxis(m_operatorController.getLeftY());}
              }, 
              new DoubleSupplier() {
                @Override
                public double getAsDouble() {return modifyAxis(m_operatorController.getLeftX());}
            }));
        
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  private void configureAutos() {
    PathPlannerTrajectory traj = PathPlanner.loadPath("Left Low Double", Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    Command driveComand = m_drivetrainSubsystem.followTrajectoryCommand(traj, true);

    HashMap<String, Command> eventMap = new HashMap<String, Command>(); 

    eventMap.put("runIntake", new RunIntake(0.9).withTimeout(3.0));
    eventMap.put("runIntakeSlow", new RunIntake(0.25).withTimeout(2.0));
    eventMap.put("lowerIntake", new LowerIntake());
    eventMap.put("raiseIntake", new RaiseIntake());
    eventMap.put("reverseConveyer", new RunConveyer(0.75).withTimeout(0.8));


    List<EventMarker> markers = traj.getMarkers();
    

    Command withEvents = new FollowPathWithEvents(driveComand, traj.getMarkers(), eventMap);

    PathPlannerTrajectory chargeTrajectory = PathPlanner.loadPath("Over The Rainbow", Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    Command chargerComand = m_drivetrainSubsystem.followTrajectoryCommand(chargeTrajectory, true);
    Command chargerWithEvents = new FollowPathWithEvents(chargerComand, chargeTrajectory.getMarkers(), eventMap);

    PathPlannerTrajectory rightTrajectory = PathPlanner.loadPath("Right Low Double", Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    Command rightCommand = m_drivetrainSubsystem.followTrajectoryCommand(rightTrajectory, true);
    Command rightWithEvents = new FollowPathWithEvents(rightCommand, rightTrajectory.getMarkers(), eventMap);

    Command autoCommand = new RunConveyer(-1).raceWith(new WaitCommand(1.4))
    .andThen(withEvents)
    .andThen(new RunConveyer(-1).raceWith(new WaitCommand(1.4)));

    Command autoCommand1 = new RunConveyer(-1).raceWith(new WaitCommand(1.4))
    .andThen(chargerWithEvents)
    .andThen(new Autobalance(BalancePoint.LEVEL));

    Command autoCommand2 = new RunConveyer(-1).raceWith(new WaitCommand(1.4))
    .andThen(rightWithEvents)
    .andThen(new RunConveyer(-1).raceWith(new WaitCommand(1.4)));


   autChooser.addOption("LeftSide", autoCommand);
   autChooser.addOption("ChargingStation", autoCommand1);
   autChooser.addOption("RightSide", autoCommand2);
   Shuffleboard.getTab("Auto")
   .add(autChooser);
  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autChooser.getSelected(); //Autos.exampleAuto(m_exampleSubsystem);
  }

  public static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.15);
    //  Chanced this value from 0.05

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

}
