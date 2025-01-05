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
  

  private SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeMotorId, MotorType.kBrushless);

  public Intake() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    intakeMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakeSpeed", 
    intakeMotor.getEncoder().getVelocity());
  }

  // start intaking
  public void setIntakeOn() {
    intakeMotor.set(IntakeConstants.intakeSpeed);
  }

  // Stop intaking
  public void setIntakeOff() {
    intakeMotor.set(0);
  }
}
