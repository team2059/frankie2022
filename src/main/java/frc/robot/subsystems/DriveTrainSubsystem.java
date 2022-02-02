// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class DriveTrainSubsystem extends SubsystemBase {

  CANSparkMax leftFrontCANSparkMax = new CANSparkMax(DriveConstants.leftFrontCANSparkMaxCANId,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftBackCANSparkMax = new CANSparkMax(DriveConstants.leftbackCANSparkMaxCANId,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightFrontCANSparkMax = new CANSparkMax(DriveConstants.rightFrontCANSparkMaxCANId,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightBackCANSparkMax = new CANSparkMax(DriveConstants.rightBackCANSparkMaxCANId,
      CANSparkMaxLowLevel.MotorType.kBrushless);

  // using built in encoders in NEO motors
  private final RelativeEncoder leftRelativeEncoder = leftFrontCANSparkMax.getEncoder();
  private final RelativeEncoder rightRelativeEncoder = rightFrontCANSparkMax.getEncoder();

  private final MotorControllerGroup leftMotorControllerGroup = new MotorControllerGroup(leftFrontCANSparkMax,
      leftBackCANSparkMax);
  private final MotorControllerGroup rightMotorControllerGroup = new MotorControllerGroup(rightFrontCANSparkMax,
      rightBackCANSparkMax);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorControllerGroup,
      rightMotorControllerGroup);

  /** Creates a new DriveTrain. */
  public DriveTrainSubsystem() {
  
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // leftMotorControllerGroup.setInverted(true);
    // rightMotorControllerGroup.setInverted(true);
    // kLinearDistancePerMotorRotation = gear
    // ratio*2*pi*Units.inchesToMeters(wheel raidus)
    // velocity is / 60 to go from meters/minute to meters/second
    // kLinearDistancePerMotorRotation and velocity is / 60 to go from meters/minute
    // to meters/second
    rightRelativeEncoder.setPositionConversionFactor(DriveConstants.kLinearDistancePerMotorRotation);
    leftRelativeEncoder.setPositionConversionFactor(DriveConstants.kLinearDistancePerMotorRotation);
    rightRelativeEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistancePerMotorRotation / 60);
    leftRelativeEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistancePerMotorRotation / 60);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_odometry.update(
    // m_gyro.getRotation2d(), m_leftEncoder.getDistance(),
    // m_rightEncoder.getDistance());

    // TODO: Find what getDistance() returns and see if it is the same as
    // getPosition()
    SmartDashboard.putNumber("Left encoder value in meters", leftRelativeEncoder.getPosition());
    SmartDashboard.putNumber("Right encoder value in meters", rightRelativeEncoder.getPosition());
    SmartDashboard.putNumber("Gyro heading in Degrees ", m_gyro.getRotation2d().getDegrees());

    m_odometry.update(m_gyro.getRotation2d(), leftRelativeEncoder.getPosition(), rightRelativeEncoder.getPosition());
  }

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // 2pr/(minutes/rotation)
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // using explciity declared encoders
    // return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(),
    // m_rightEncoder.getRate());

    return new DifferentialDriveWheelSpeeds(leftRelativeEncoder.getVelocity(), rightRelativeEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotorControllerGroup.setVoltage(leftVolts);
    rightMotorControllerGroup.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    // m_leftEncoder.reset();
    // m_rightEncoder.reset();

    // TODO: Makre sure .setPosition(0) actually resets encoder position to 0
    leftRelativeEncoder.setPosition(0);
    rightRelativeEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    // return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;

    // TODO: Find what getDistance() returns and see if it is the same as
    // getPosition()
    return (leftRelativeEncoder.getPosition() + rightRelativeEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */

  // public Encoder getLeftEncoder() {
  // return m_leftEncoder;
  // }

  // TODO: See what difference is in returning Encoder object vs RelativeEncoder
  // object
  public RelativeEncoder getLeftEncoder() {
    return leftRelativeEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */

  // public Encoder getRightEncoder() {
  // return m_rightEncoder;
  // }

  // TODO: See what difference is in returning Encoder object vs RelativeEncoder
  // object
  public RelativeEncoder getRightEncoder() {
    return rightRelativeEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
