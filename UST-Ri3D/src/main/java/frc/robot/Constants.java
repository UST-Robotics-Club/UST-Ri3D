// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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
  public static class OperatorConstants {
    public static final int
    kDriverControllerPort = 0;
  }

  public static class ElevatorConstants {
    public static final int
    motorID = 7,
    limitSwitchID = 0; // TODO: Add real value
    public static final double
    gearRatio = 0.0,      // TODO: Add real value
    wheelDiameter = 0.0,  // TODO: Add real value
    // Rotations to meters
    positionConversionFactor = gearRatio * Math.PI * wheelDiameter,

    p = 0.0,              // TODO: Add real value
    i = 0.0,              // TODO: Add real value
    d = 0.0,              // TODO: Add real value
    minOutput = 0.0,      // TODO: Add real value
    maxOutput = 0.0,      // TODO: Add real value

    // Coral level heights in meters from elevator base
    elevatorBaseHeight = 0.0; // TODO: Add real value
    public static final double[] levels = {
elevatorBaseHeight,
      0.46 - elevatorBaseHeight, // Adjust as needed
      0.81 - elevatorBaseHeight,
      1.21 - elevatorBaseHeight,
      1.83 - elevatorBaseHeight
    };
  }

  public static class DriveConstants {
    public static final int frontLeftID = 1,
        frontRightID = 2,
        backLeftID = 3,
        backRightID = 4;
    public static final double gearRatio = 10.71,
        wheelDiameter = Units.inchesToMeters(6),
        rotationConversionFactor = (1 / gearRatio) * wheelDiameter * Math.PI;
    // Locations of the wheels relative to the robot center. Not correct right now
    public static Translation2d frontLeftLocation = new Translation2d(0.381, 0.381),
        frontRightLocation = new Translation2d(0.381, -0.381),
        backLeftLocation = new Translation2d(-0.381, 0.381),
        backRightLocation = new Translation2d(-0.381, -0.381);
  }

  public static class IntakeConstants {
    public static final int 
      intakeMotorId = 5;
    public static final double 
      intakeSpeed = 0.9;
  }

  public static class OuttakeConstants {
    public static final int
      armMotorId = 6,
      extensionMotorId = 7,
      clawMotorId = 8;
    
    public static final double
      p = 0.0,
      i = 0.0,
      d = 0.0,
      armEncoderOffset = 0.0,
      gearRatio = 0.0,      // TODO: Add real value
      wheelDiameter = 0.0,  // TODO: Add real value
      // Rotations to meters
      positionConversionFactor = gearRatio * Math.PI * wheelDiameter;

    public static final double[] angles = {
      armEncoderOffset,
      125 + armEncoderOffset,
      125 + armEncoderOffset,
      125 + armEncoderOffset,
      90 + armEncoderOffset
    };
  }
}
