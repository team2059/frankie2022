// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class DriveTrainSubsystem extends SubsystemBase {

  CANSparkMax leftFrontCANSparkMax = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftBackCANSparkMax = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightFrontCANSparkMax = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightBackCANSparkMax = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);

  RelativeEncoder leftRelativeEncoder = leftFrontCANSparkMax.getEncoder();
  RelativeEncoder rightRelativeEncoder = rightFrontCANSparkMax.getEncoder();

  MotorControllerGroup leftMotorControllerGroup = new MotorControllerGroup(leftFrontCANSparkMax, leftBackCANSparkMax);
  MotorControllerGroup rightMotorControllerGroup = new MotorControllerGroup(rightFrontCANSparkMax, rightBackCANSparkMax);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorControllerGroup, rightMotorControllerGroup);

  /** Creates a new DriveTrain. */
  public DriveTrainSubsystem() {
    //m_right.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    differentialDrive.arcadeDrive(xSpeed, zRotation);
  }
}
