// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class AutoConstants {
        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // for paths that are coded. in pathweaver we can set these values
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    }

    public static final class DriveConstants {
        // CAN ID Ports
        public static int leftFrontCANSparkMaxCANId = 2;
        public static int leftbackCANSparkMaxCANId = 1;
        public static int rightFrontCANSparkMaxCANId = 3;
        public static int rightBackCANSparkMaxCANId = 4;

        //
        // ratio*2*pi*Units.inchesToMeters(wheel raidus)
        public static final double kFrankieGearRatio = 12.755;
        public static final double kFrankieWheelRadiusInches = 3;

        // If we get the number of ticks, say 10000, we want to multiply that times
        // 1/kLinearDistancePerMotorRotation to get number of inches, not 10000 times
        // kLinearDistancePerMotorRotation
        
        public static final double kLinearDistancePerMotorRotation = (Units
                .inchesToMeters(1 / (kFrankieGearRatio * 2 * Math.PI
                        * Units.inchesToMeters(kFrankieWheelRadiusInches)) * 10));

        
        public static final double ksVolts = 0.1219;
        public static final double kvVoltSecondsPerMeter = 3.343;
        public static final double kaVoltSecondsSquaredPerMeter = 1.0356;
        public static final double kPDriveVel = 2.2662;

        // 22.75 inches equals 0.57785. trackwidth is horizontal distance between the
        // wheels
        public static final double kTrackwidthMeters = Units.inchesToMeters(23);
        // DifferentialDriveKinematics allows us to use the trackwidth to convert from
        // chassis speeds to wheel speeds. As elsewhere, we keep our units in meters.
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);
    }

}
