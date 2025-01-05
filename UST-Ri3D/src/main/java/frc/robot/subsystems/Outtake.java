// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

public class Outtake extends SubsystemBase {
  public Outtake() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    arm.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    extension.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    claw.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
  }

  SparkMax arm = new SparkMax(OuttakeConstants.armMotorId, null);
  SparkMax extension = new SparkMax(OuttakeConstants.extensionMotorId, null);
  SparkMax claw = new SparkMax(OuttakeConstants.clawMotorId, null);

  AnalogInput armEncoder = new AnalogInput(0);
  RelativeEncoder extensionEncoder = extension.getEncoder();

  ArmFeedforward feedforwardArm = new ArmFeedforward(0, 0, 0); // TODO: Find feedforward constants using SysID
  TrapezoidProfile.Constraints constraintsArm = new TrapezoidProfile.Constraints(0, 0); // TODO: Find trapezoid profile constraints
  ProfiledPIDController controllerArm = new ProfiledPIDController(OuttakeConstants.p, OuttakeConstants.i, OuttakeConstants.d, constraintsArm);

  ElevatorFeedforward feedforwardExtension = new ElevatorFeedforward(0, 0, 0); // TODO: Find feedforward constants using SysID
  TrapezoidProfile.Constraints constraintsExtension = new TrapezoidProfile.Constraints(0, 0); // TODO: Find trapezoid profile constraints
  ProfiledPIDController controllerExtension = new ProfiledPIDController(OuttakeConstants.p, OuttakeConstants.i, OuttakeConstants.d, constraintsExtension);

  int currentLevel = 0;

  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) {
      angleArmToSetpoint(OuttakeConstants.angles[currentLevel]);
    }
  }

  double getArmPosition() {
    return armEncoder.getVoltage() / RobotController.getCurrent3V3() * 360;
  }

  double getExtensionPosition() {
    return extensionEncoder.getPosition();
  }

  /*
   * 
   */
  public void angleArmToSetpoint(double setpoint) {
    arm.setVoltage(
      controllerArm.calculate(getArmPosition())
        + feedforwardArm.calculate(getArmPosition(), controllerArm.getSetpoint().velocity));
  }

  public void extendArmToSetpoint(double setpoint) {
    extension.setVoltage(
      controllerExtension.calculate(getArmPosition())
        + feedforwardExtension.calculate(controllerExtension.getSetpoint().velocity));
  }

  public void setLevel(int index) {
    currentLevel = index;
  }
}
