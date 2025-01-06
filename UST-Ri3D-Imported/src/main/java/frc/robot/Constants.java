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
    motorID = 10,
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

    kS = 0.0, // TODO: Add real value
    kG = 0.0, // TODO: Add real value
    kV = 0.0, // TODO: Add real value
    maxVel = 0.0, // TODO: Add real value
    maxAccel = 0.0, // TODO: Add real value

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
    public static final int frontLeftID = 2,
        frontRightID = 1,
        backLeftID = 3,
        backRightID = 4;
    public static final double gearRatio = 10.71,
        wheelDiameter = Units.inchesToMeters(6.35),
        rotationConversionFactor = (1 / gearRatio) * wheelDiameter * Math.PI,
        // RPM to meters per second
        velocityConversionFactor = rotationConversionFactor / 60,
        maxWheelVelocity = 3.33;
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
      extensionMotorId = 9,
      clawMotorId = 8,
      absEncoderId = 0; // TODO: Add real value
    
    public static final double
      pArm = 0.0, // TODO: Add real value
      iArm = 0.0, // TODO: Add real value
      dArm = 0.0, // TODO: Add real value
      kS = 0.0, // TODO: Add real value
      kG = 0.0, // TODO: Add real value
      kV = 0.0, // TODO: Add real value
      maxVel = 0.0, // TODO: Add real value
      maxAccel = 0.0, // TODO: Add real value
      armEncoderOffset = 0.0, // TODO: Add real value
      armGearRatio = 0.0,      // TODO: Add real value
      wheelDiameter = 0.0,  // TODO: Add real value
      // Motor Rotations to Arm Degrees
      angleConversionFactor = (1/armGearRatio) * 360;

    public static final double[] angles = {
      armEncoderOffset,
      125 + armEncoderOffset,
      90 + armEncoderOffset
    };
  }
}
