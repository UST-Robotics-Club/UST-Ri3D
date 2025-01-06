// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  ADIS16470_IMU imu = new ADIS16470_IMU();
  SparkMax frontLeft = new SparkMax(DriveConstants.frontLeftID, MotorType.kBrushless);
  SparkMax frontRight = new SparkMax(DriveConstants.frontRightID, MotorType.kBrushless);
  SparkMax backLeft = new SparkMax(DriveConstants.backLeftID, MotorType.kBrushless);
  SparkMax backRight = new SparkMax(DriveConstants.backRightID, MotorType.kBrushless);
  
  // Creating my kinematics object using the wheel locations.
  private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
      DriveConstants.frontLeftLocation, DriveConstants.frontRightLocation, 
      DriveConstants.backLeftLocation,  DriveConstants.backRightLocation);
  private Field2d fieldDisplay = new Field2d();
  // Rotations to meters traveled
  private MecanumDriveOdometry odometry = new MecanumDriveOdometry(
      kinematics,
      getRotation(),
      getWheelPositions(),
      new Pose2d(0, 0, new Rotation2d()));

  public Drivetrain() {
    resetGyro(0);
    SmartDashboard.putData(fieldDisplay);
  }

  @Override
  public void periodic() {
    odometry.update(getRotation(), getWheelPositions());
    fieldDisplay.setRobotPose(getPose());
    SmartDashboard.putNumber("FrontRightSpeed", 
    frontRight.getEncoder().getVelocity() * DriveConstants.rotationConversionFactor);
    SmartDashboard.putNumber("X pos", getPose().getX());
    SmartDashboard.putNumber("Y pos", getPose().getY());

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

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(frontLeft.getEncoder().getPosition() * DriveConstants.rotationConversionFactor,
        frontRight.getEncoder().getPosition() * DriveConstants.rotationConversionFactor,
        backLeft.getEncoder().getPosition() * DriveConstants.rotationConversionFactor,
        backRight.getEncoder().getPosition() * DriveConstants.rotationConversionFactor);
  }

}
