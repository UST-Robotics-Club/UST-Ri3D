// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final int frontLeftID = 1,
        frontRightID = 2,
        backLeftID = 3,
        backRightID = 4;
    public static final double gearRatio = 10.71,
        wheelDiameter = 0.15,
        rotationConversionFactor = (1 / gearRatio) * wheelDiameter * Math.PI;
    // Locations of the wheels relative to the robot center. Not correct right now
    public static Translation2d frontLeftLocation = new Translation2d(0.381, 0.381),
        frontRightLocation = new Translation2d(0.381, -0.381),
        backLeftLocation = new Translation2d(-0.381, 0.381),
        backRightLocation = new Translation2d(-0.381, -0.381);
  }

  public static class IntakeConstants {
    public static final int deployerId = 5,
        intakeMotorId = 6,
        internalReleaserId = 7,
        releaseTime = 1000;
    public static final double deployedPosition = 100,
        retractedPosition = 0,
        intakeSpeed = 0.9,
        releaserSpeed = 0.5;
  }
}
