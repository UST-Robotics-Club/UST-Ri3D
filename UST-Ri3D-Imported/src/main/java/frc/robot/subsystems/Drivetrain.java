// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  CANSparkMax frontLeft = new CANSparkMax(DriveConstants.frontLeftID, MotorType.kBrushless);
  CANSparkMax frontRight = new CANSparkMax(DriveConstants.frontRightID, MotorType.kBrushless);
  CANSparkMax backLeft = new CANSparkMax(DriveConstants.backLeftID, MotorType.kBrushless);
  CANSparkMax backRight = new CANSparkMax(DriveConstants.backRightID, MotorType.kBrushless);
  
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
    frontLeft.setIdleMode(IdleMode.kBrake);    
    frontRight.setIdleMode(IdleMode.kBrake);
    backLeft.setIdleMode(IdleMode.kBrake);
    backRight.setIdleMode(IdleMode.kBrake);

    frontRight.setInverted(true);
    backRight.setInverted(true);
    frontLeft.setInverted(false);
    backLeft.setInverted(false);

    frontLeft.setOpenLoopRampRate(0.25);    
    frontRight.setOpenLoopRampRate(0.25);
    backLeft.setOpenLoopRampRate(0.25);
    backRight.setOpenLoopRampRate(0.25);

  }

  @Override
  public void periodic() {
    odometry.update(getRotation(), getWheelPositions());
    fieldDisplay.setRobotPose(getPose());
    SmartDashboard.putNumber("FrontRightSpeed", frontRight.getEncoder().getVelocity() * DriveConstants.velocityConversionFactor);
    SmartDashboard.putNumber("FrontLeftSpeed", frontLeft.getEncoder().getVelocity() * DriveConstants.velocityConversionFactor);
    SmartDashboard.putNumber("FrontLeftCurr", frontLeft.getOutputCurrent());

    SmartDashboard.putNumber("X pos", getPose().getX());
    SmartDashboard.putNumber("Y pos", getPose().getY());

  }

  public void resetPosition(double x, double y) {
    odometry.resetPosition(getRotation(), getWheelPositions(), new Pose2d(x, y, getRotation()));
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
     // 200 units
    wheelSpeeds.desaturate(DriveConstants.maxWheelVelocity);
    frontLeft.set(wheelSpeeds.frontLeftMetersPerSecond);
    frontRight.set(wheelSpeeds.frontRightMetersPerSecond);
    backLeft.set(wheelSpeeds.rearLeftMetersPerSecond);
    backRight.set(wheelSpeeds.rearRightMetersPerSecond);
  }

  public void drive(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    frontLeft.getPIDController().setReference(wheelSpeeds.frontLeftMetersPerSecond, ControlType.kVelocity);
    frontRight.getPIDController().setReference(wheelSpeeds.frontRightMetersPerSecond, ControlType.kVelocity);
    backLeft.getPIDController().setReference(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kVelocity);
    backRight.getPIDController().setReference(wheelSpeeds.rearRightMetersPerSecond, ControlType.kVelocity);
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(frontLeft.getEncoder().getPosition() * DriveConstants.rotationConversionFactor,
        frontRight.getEncoder().getPosition() * DriveConstants.rotationConversionFactor,
        backLeft.getEncoder().getPosition() * DriveConstants.rotationConversionFactor,
        backRight.getEncoder().getPosition() * DriveConstants.rotationConversionFactor);
  }

}
