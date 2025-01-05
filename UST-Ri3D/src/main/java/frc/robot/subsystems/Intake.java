// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  

  private SparkMax deployer = new SparkMax(IntakeConstants.deployerId, MotorType.kBrushless);
  private SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeMotorId, MotorType.kBrushless);
  private SparkMax internalReleaser = new SparkMax(IntakeConstants.internalReleaserId, MotorType.kBrushless);
  private long timeToStopRelease = 0;

  public Intake() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    deployer.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    intakeMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    internalReleaser.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
  }
  @Override
  public void periodic() {
    if(timeToStopRelease <= System.currentTimeMillis()) {
      internalReleaser.set(IntakeConstants.releaserSpeed);
    } else {
      internalReleaser.set(0);
    }
    
    SmartDashboard.putNumber("deployerPos", 
    deployer.getEncoder().getPosition() * DriveConstants.rotationConversionFactor);

    SmartDashboard.putNumber("intakeSpeed", 
    intakeMotor.getEncoder().getVelocity());

    SmartDashboard.putNumber("releaseSpeed", 
    internalReleaser.getEncoder().getVelocity());
  }
  // Move intake to deployed position
  public void deploy() {
    deployer.getClosedLoopController().setReference(IntakeConstants.deployedPosition, ControlType.kPosition);
  }
  // Move intake to retracted position
  public void retract() {
    deployer.getClosedLoopController().setReference(IntakeConstants.retractedPosition, ControlType.kPosition);
  }
  // start intaking
  public void setIntakeOn() {
    intakeMotor.set(IntakeConstants.intakeSpeed);
  }

  // Stop intaking
  public void setIntakeOff() {
    intakeMotor.set(0);
  }

  // Run the release motor for a default time
  public void releaseCoral() {
    timeToStopRelease = System.currentTimeMillis() + IntakeConstants.releaseTime;
  }

  // Run the release motor for a custom time
  public void releaseCoral(int time) {
    timeToStopRelease = System.currentTimeMillis() + time;
  }
}
