// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  // private DriveTrainSubsystem driveTrainSubsystem;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.getDriveTrainSubsystem().zeroHeading();
    m_robotContainer.getDriveTrainSubsystem().resetEncoders();

    // m_robotContainer.getDriveTrainSubsystem().zeroHeading();
    // m_robotContainer.getDriveTrainSubsystem().resetEncoders();

  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    m_robotContainer.getDriveTrainSubsystem().zeroHeading();
    m_robotContainer.getDriveTrainSubsystem().resetEncoders();

    try {
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    // if (driveTrainSubsystem.getHeading() != 0) {
    // driveTrainSubsystem.zeroHeading();
    // }

  }

  @Override
  public void teleopInit() {
    m_robotContainer.getDriveTrainSubsystem().zeroHeading();
    m_robotContainer.getDriveTrainSubsystem().resetEncoders();

    Rotation2d thetaPose = m_robotContainer.getDriveTrainSubsystem().navX.getRotation2d();
    DifferentialDriveOdometry myDriveOdometry = m_robotContainer.getDriveTrainSubsystem().getOdometry();
    myDriveOdometry.resetPosition(new Pose2d(), thetaPose);

    // if(null == driveTrainSubsystem) {
    // SmartDashboard.putString("driveTrainSubsystem" , "is NULLL");
    // }
    // driveTrainSubsystem.zeroHeading();
    // driveTrainSubsystem.resetEncoders();

    // Rotation2d thetaPose = driveTrainSubsystem.navX.getRotation2d();

    // DifferentialDriveOdometry myDriveOdometry = null;
    // try {
    // myDriveOdometry = driveTrainSubsystem.getOdometry();
    // }catch (NullPointerException ex){
    // SmartDashboard.putString("myDriveOdometry" , "try catch NULLL");
    // ex.printStackTrace();

    // }

    // if(null == myDriveOdometry) {
    // SmartDashboard.putString("myDriveOdometry" , "is NULLL");
    // }

    // try{
    // myDriveOdometry.resetPosition(new Pose2d(), thetaPose);
    // }catch(NullPointerException exception){
    // SmartDashboard.putString("myDriveOdometry" , "cannot reset pos");
    // }
    //

    // driveTrainSubsystem.getOdometry().resetPosition(new Pose2d(), thetaPose);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
