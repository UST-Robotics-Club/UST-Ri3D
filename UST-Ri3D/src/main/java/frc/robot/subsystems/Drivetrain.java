// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.OperatorDrive;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  ADIS16470_IMU imu = new ADIS16470_IMU();
  SparkMax frontLeft = new SparkMax(DriveConstants.frontLeftID, MotorType.kBrushless);
  SparkMax frontRight = new SparkMax(DriveConstants.frontRightID, MotorType.kBrushless);
  SparkMax backLeft = new SparkMax(DriveConstants.backLeftID, MotorType.kBrushless);
  SparkMax backRight = new SparkMax(DriveConstants.backRightID, MotorType.kBrushless);

  // Locations of the wheels relative to the robot center.
  Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  Translation2d backRightLocation = new Translation2d(-0.381, -0.381);
  // Creating my kinematics object using the wheel locations.
  private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  // Rotations to meters traveled
  private final double positionConversionFactor = (1 / 10.71) * Math.PI * 0.15;
  private MecanumDriveOdometry odometry = new MecanumDriveOdometry(
      kinematics,
      getRotation(),
      getWheelPositions(),
      new Pose2d(0, 0, new Rotation2d()));

  public Drivetrain() {
    resetGyro(0);
  }

  public void resetPosition(double x, double y) {
    odometry.resetTranslation(new Translation2d(x, y));
  }

  public void resetGyro(double angle) {
    imu.setGyroAngleZ(angle);
  }

  public double getAngleDegrees() {
    return imu.getAngle();
  }

  public double getAngleRadians() {
    return Math.toRadians(getAngleDegrees());
  }

  public Rotation2d getRotation() {
    return new Rotation2d(getAngleRadians());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // Robot relative. X = forward, y = left, rot = counterclockise
  public void drive(double x, double y, double rot) {
    drive(new ChassisSpeeds(x, y, rot));
  }

  // Uses percents instead of meters per second
  public void drivePercent(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    frontLeft.set(wheelSpeeds.frontLeftMetersPerSecond);
    frontRight.set(wheelSpeeds.frontRightMetersPerSecond);
    backLeft.set(wheelSpeeds.rearLeftMetersPerSecond);
    backRight.set(wheelSpeeds.rearRightMetersPerSecond);
  }

  public void drive(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    frontLeft.getClosedLoopController().setReference(wheelSpeeds.frontLeftMetersPerSecond, ControlType.kVelocity);
    frontRight.getClosedLoopController().setReference(wheelSpeeds.frontRightMetersPerSecond, ControlType.kVelocity);
    backLeft.getClosedLoopController().setReference(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kVelocity);
    backRight.getClosedLoopController().setReference(wheelSpeeds.rearRightMetersPerSecond, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    odometry.update(getRotation(), getWheelPositions());
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(frontLeft.getEncoder().getPosition() * positionConversionFactor,
        frontRight.getEncoder().getPosition() * positionConversionFactor,
        backLeft.getEncoder().getPosition() * positionConversionFactor,
        backRight.getEncoder().getPosition() * positionConversionFactor);
  }

}
