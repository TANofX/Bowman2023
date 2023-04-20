// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autobalance;
import frc.robot.commands.CloseGripper;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.LightUpCone;
import frc.robot.commands.LightUpCube;
import frc.robot.commands.LowerIntake;
import frc.robot.commands.ManualArm;
import frc.robot.commands.MoveArmMeters;
import frc.robot.commands.MoveArmToArmPosition;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.OpenGripper;
import frc.robot.commands.RaiseIntake;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GreekArm;
import frc.robot.commands.RunConveyer;
import frc.robot.commands.RunIntake;
import frc.robot.commands.StopArm;
import frc.robot.commands.ZeroYaw;
import frc.robot.commands.Autobalance.BalancePoint;
import frc.robot.subsystems.ArmPositions;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.ConveyerBelt;
import frc.robot.subsystems.EverybotIntake;

import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  public static final EverybotIntake m_intake = new EverybotIntake();
  public static final ConveyerBelt m_conveyer = new ConveyerBelt();
  public static final CandleSubsystem m_candle = new CandleSubsystem();

  
  private static HashMap<String, Command> eventMap = new HashMap<String, Command>(); 

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
    configureShuffleBoardButtons();
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

    m_operatorController.y().onTrue(new StopArm());

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

    m_driverController.rightTrigger().whileTrue(new RunIntake(.75));
    m_driverController.button(7).whileTrue(new RunIntake(.5));
    m_driverController.leftTrigger().onTrue(new MoveArmToArmPosition(ArmPositions.SLAM_JAM).withTimeout(0.5)
                                            .andThen(new OpenGripper())
                                            .andThen(new MoveArmToArmPosition(ArmPositions.PICK_UP))
                                            .andThen(new InstantCommand(() -> {m_conveyer.closeConveyer();})));

                                            // m_driverController.leftTrigger().onTrue(new MoveArmMeters(0.0, -0.2, 3.0).withTimeout(0.5)
    //                                         .andThen(new OpenGripper())
    //                                         .andThen(new MoveArmMeters(0.0, 0.2, 5.0).withTimeout(0.5))
    //                                         .andThen(new MoveArmToArmPosition(ArmPositions.PICK_UP))
    //                                         .andThen(new InstantCommand(() -> {m_conveyer.closeConveyer();})));

    m_operatorController.leftTrigger().onTrue(new MoveArmToArmPosition(ArmPositions.STOW_HIGH));

    m_operatorController.povLeft().onTrue(new LightUpCone());
    m_operatorController.povRight().onTrue(new LightUpCube());

    m_driverController.leftBumper().onTrue(new RaiseIntake());
    m_driverController.rightBumper().onTrue(new LowerIntake());

    m_driverController.povUp().whileTrue(new Autobalance(Autobalance.BalancePoint.FORWARD));
    m_driverController.povDown().whileTrue(new Autobalance(Autobalance.BalancePoint.BACKWARD));
    m_driverController.a().whileTrue(new Autobalance(Autobalance.BalancePoint.LEVEL));

  m_operatorController.a().onTrue(new InstantCommand(() -> {m_conveyer.openConveyer();}).andThen(new WaitCommand(0.125)).andThen(new MoveArmToArmPosition(ArmPositions.MID_SCORE)));
  m_operatorController.x().onTrue(new InstantCommand(() -> {m_conveyer.openConveyer();}).andThen(new WaitCommand(0.125)).andThen(new MoveArmToArmPosition(ArmPositions.HIGH_SCORE)));
  m_operatorController.b().onTrue((new MoveArmToArmPosition(ArmPositions.PICK_UP)).andThen(new InstantCommand(() -> {m_conveyer.closeConveyer();})));
  m_operatorController.rightBumper().onTrue(new InstantCommand(() -> {m_conveyer.closeConveyer();}).andThen(new MoveArmToArmPosition(ArmPositions.HOME)));






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
                      return -modifyAxis(m_driverController.getLeftY());
                  }
            },
            new DoubleSupplier() {
              @Override
              public double getAsDouble() {
                  return -modifyAxis(m_driverController.getLeftX());
              }
            },
            new DoubleSupplier() {
              public double getAsDouble() {
                return -modifyAxis(m_driverController.getRightX());
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
    ShuffleboardTab currentTab = Shuffleboard.getTab("Other Buttons");

    currentTab.add("zeroGyro", new InstantCommand (() -> {m_drivetrainSubsystem.zeroGyroscope();}) );
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  private void configureAutos() {
    eventMap.put("runIntake", new RunIntake(0.5).withTimeout(3.0));
    eventMap.put("runIntakeSlow", new RunIntake(0.25).withTimeout(2.0));
    eventMap.put("lowerIntake", new LowerIntake());
    eventMap.put("raiseIntake", new RaiseIntake());
    eventMap.put("reverseConveyer", new RunConveyer(0.75).withTimeout(0.8));
    eventMap.put("runConveyer", new RunConveyer(0.75).withTimeout(3.0));

    // Auto Choice Options
   autChooser.addOption("Place High", new ZeroYaw().andThen(placeHigh())
                        .andThen(new MoveArmToArmPosition(ArmPositions.HOME)));
   autChooser.addOption("Middle Charge High", placeHighBalance("Middle Red Charge High"));

    //BLUE AUTOS
   autChooser.addOption("Left Blue Double High", placeHighBeforeAndAfter("Left Blue Double High"));
   //autChooser.addOption("Left Blue Double Low", doubleScoreLow("Left Blue Double Low"));
 //  autChooser.addOption("Left Blue Charge High", placeHighBalance("Left Blue Charge High"));
   //autChooser.addOption("Middle Blue Charge High", placeHighBalance("Middle Blue Charge High"));
   autChooser.addOption("Right Blue Double High", placeHighBeforeAndAfter("Right Blue Double High"));
   //autChooser.addOption("Right Blue Double Low", doubleScoreLow("Right Blue Double Low"));
   //autChooser.addOption("Right Blue Charge High", placeHighBalance("Right Blue Charge High"));
    //RED AUTOS
   autChooser.addOption("Left Red Double High", placeHighBeforeAndAfter("Left Red Double High"));
   //autChooser.addOption("Left Red Double Low", doubleScoreLow("Left Red Double Low"));
   //autChooser.addOption("Left Red Charge High", placeHighBalance("Left Red Charge High"));
   autChooser.addOption("Right Red Double High", placeHighBeforeAndAfter("Right Red Double High"));
   //autChooser.addOption("Right Red Double Low", doubleScoreLow("Right Red Double Low"));
   //autChooser.addOption("Right Red Charge High", placeHighBalance("Right Red Charge High"));
   
   Shuffleboard.getTab("Auto")
   .add(autChooser);
  }

  private Command doubleScoreLow(String pathName) {
    PathPlannerTrajectory redRightTrajectory = PathPlanner.loadPath(pathName, Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    Command redRightCommand = m_drivetrainSubsystem.followTrajectoryCommand(redRightTrajectory, true);
    Command redRightWithEvents = new FollowPathWithEvents(redRightCommand, redRightTrajectory.getMarkers(), eventMap);

    Command autoCommand5 = new RunConveyer(-1).raceWith(new WaitCommand(1.4))
    .andThen(redRightWithEvents)
    .andThen(new RunConveyer(-1).raceWith(new WaitCommand(1.4)));
    return autoCommand5;
  }

  private Command placeHighScoreLow(String pathName) {
    PathPlannerTrajectory blueLeftTraj = PathPlanner.loadPath(pathName, Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    Command blueLeftCommand = m_drivetrainSubsystem.followTrajectoryCommand(blueLeftTraj, true);
    Command blueLeftWithEvents = new FollowPathWithEvents(blueLeftCommand, blueLeftTraj.getMarkers(), eventMap);
    return placeHigh()
    .andThen(new MoveArmToArmPosition(ArmPositions.HOME)
    .alongWith(blueLeftWithEvents))
    .andThen(new RunConveyer(-1).raceWith(new WaitCommand(1.4)));
  }

  private Command placeHighBeforeAndAfter(String pathName) {
    PathPlannerTrajectory currentTrajectory = PathPlanner.loadPath(pathName, Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    Command followCommand = m_drivetrainSubsystem.followTrajectoryCommand(currentTrajectory, true);
    Command followWithEvents = new FollowPathWithEvents(followCommand, currentTrajectory.getMarkers(), eventMap);
    return placeHigh()
    .andThen(new InstantCommand(() -> {m_conveyer.openConveyer();}))
    .andThen((new MoveArmToArmPosition(ArmPositions.PICK_UP).andThen(new InstantCommand(() -> {m_conveyer.closeConveyer();})))
    .alongWith(followWithEvents)
    )
    .andThen(new CloseGripper())
    .andThen(new InstantCommand(() -> {m_conveyer.openConveyer();}).andThen(new WaitCommand(0.125)))
    .andThen(new MoveArmToArmPosition(ArmPositions.HIGH_SCORE).raceWith(new WaitCommand(2.4))
    .andThen(new OpenGripper()))
    .andThen(new MoveArmToArmPosition(ArmPositions.HOME));
  }

  private Command placeHighBalance(String pathName) {
    List<PathPlannerTrajectory> blueHighCharge = PathPlanner.loadPathGroup(pathName, new PathConstraints(1.5, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    Command driveCommand = null;
    
    for (PathPlannerTrajectory p : blueHighCharge) {
    Command blueChargeCommand = m_drivetrainSubsystem.followTrajectoryCommand(p, driveCommand == null);
    Command blueChargeWithEvents = new FollowPathWithEvents(blueChargeCommand, p.getMarkers(), eventMap);
    if (driveCommand == null) {
      driveCommand = new WaitCommand(1)
      .andThen(blueChargeWithEvents);
    }
    else {
      driveCommand = driveCommand
      .andThen(new WaitCommand(0.75)
      .andThen(blueChargeWithEvents));
    }
  }
    return placeHigh()
    .andThen(new MoveArmToArmPosition(ArmPositions.PICK_UP)
    .alongWith(driveCommand))
    .andThen(new Autobalance(BalancePoint.LEVEL));
  }

  private Command placeHigh() {
    return new MoveArmToArmPosition(ArmPositions.PRE_PRE_PICKUP).raceWith(new WaitCommand(2.4))
    .andThen(new OpenGripper());
  }

  private void configureShuffleBoardButtons() {
    ShuffleboardTab tab = Shuffleboard.getTab("Arm Control");
    tab.add("Home", new MoveArmToArmPosition(ArmPositions.HOME));
    tab.add("Safe Transition", new MoveArmToArmPosition(ArmPositions.SAFE_TRANSITION));
    tab.add("Pre Pick Up", new MoveArmToArmPosition(ArmPositions.PRE_PICKUP));
    tab.add("Pick Up", new MoveArmToArmPosition(ArmPositions.PICK_UP));
    tab.add("Mid Score", new MoveArmToArmPosition(ArmPositions.MID_SCORE));
    tab.add("High Score", new MoveArmToArmPosition(ArmPositions.HIGH_SCORE));


  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autChooser.getSelected(); //Autos.exampleAuto(m_exampleSubsystem);
  }

  public static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      // if (value > 0.0) {
      //   return (value - deadband) / (1.0 - deadband);
      // } else {
      //   return (value + deadband) / (1.0 - deadband);
      // }
      return value;
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
