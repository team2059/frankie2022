// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Vision;

public class AutoAlign extends CommandBase {
  private final Vision vision;
  private final DriveTrainSubsystem driveTrainSubsystem;
  private double speed = 0.0;
  /** Creates a new AutoAlign. */
  public AutoAlign(Vision vision, DriveTrainSubsystem driveTrainSubsystem) {
    this.vision = vision;
    this.driveTrainSubsystem = driveTrainSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision, driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Starting AutoAlign");
    System.out.println(vision.getTargetAngle());
    speed = vision.getTargetAngle() * 0.075;

    driveTrainSubsystem.arcadeDrive(0, -(speed*0.5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
