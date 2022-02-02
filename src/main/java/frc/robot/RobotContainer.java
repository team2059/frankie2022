// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCmd;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public static Joystick joyStick = new Joystick(0);
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();

  // private final TeleopDriveCommand teleopDriveCommand = new
  // TeleopDriveCommand(driveTrainSubsystem);

  Command autonomousCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveTrainSubsystem.setDefaultCommand(new DriveCmd(driveTrainSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  public Command getAutonomousCommand() throws IOException {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        10);

    // Create config for trajectory

    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/S.wpilib.json");
    Trajectory trajectory = new Trajectory();
    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // // Start at the origin facing the +X direction
    // new Pose2d(0, 0, new Rotation2d(0)),
    // // Pass through these two interior waypoints, making an 's' curve path
    // List.of(new Translation2d(1, 1), new Translation2d(-1, 2)),
    // // End 3 meters straight ahead of where we started, facing forward
    // new Pose2d(3, 0, new Rotation2d(0)),
    // // Pass config
    // config);

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        driveTrainSubsystem::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        driveTrainSubsystem::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        driveTrainSubsystem::tankDriveVolts,
        driveTrainSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    driveTrainSubsystem.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end via break mode to ensure no
    // voltage and then set motors to idle mode for teleOp.
    return ramseteCommand.andThen(() -> driveTrainSubsystem.setBreakMode())
        .andThen(() -> driveTrainSubsystem.setIdleMode());

  }

}
