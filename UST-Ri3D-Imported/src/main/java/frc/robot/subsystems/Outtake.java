// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

public class Outtake extends SubsystemBase {
  public Outtake() {
    arm.setIdleMode(IdleMode.kBrake);
    claw.setIdleMode(IdleMode.kBrake);
    armEncoder.setDutyCycleRange(0, 360);
  }

  CANSparkMax arm = new CANSparkMax(OuttakeConstants.armMotorId, MotorType.kBrushless);
  CANSparkMax claw = new CANSparkMax(OuttakeConstants.clawMotorId, MotorType.kBrushless);

  DutyCycleEncoder armEncoder = new DutyCycleEncoder(OuttakeConstants.absEncoderId);

  ArmFeedforward feedforwardArm = new ArmFeedforward(OuttakeConstants.kS, OuttakeConstants.kG, OuttakeConstants.kV);
  TrapezoidProfile.Constraints constraintsArm = new TrapezoidProfile.Constraints(OuttakeConstants.maxVel, OuttakeConstants.maxAccel);
  ProfiledPIDController controllerArm = new ProfiledPIDController(OuttakeConstants.pArm, OuttakeConstants.iArm, OuttakeConstants.dArm, constraintsArm);

  int currentAngle = 0;

  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) {
      angleArmToSetpoint(OuttakeConstants.angles[currentAngle]);
    }
  }

  double getArmPosition() {
    return armEncoder.get();
  }

  public Command grabCoral() {
    return runOnce(
      () -> {
        setAngle(0);
        claw.set(0.5);
        new Thread(() -> {
          try {
            Thread.sleep(1000);
            claw.set(0);
          } catch (Exception e) {}
        }).start();});
  }

  public Command dropCoral() {
    return runEnd(
      () -> {
        claw.set(-0.5);
      },
      () -> {
        claw.set(0);
      }
    );
  }

  /*
   * 
   */
  public void angleArmToSetpoint(double setpoint) {
    arm.setVoltage(
      controllerArm.calculate(getArmPosition())
        + feedforwardArm.calculate(getArmPosition(), controllerArm.getSetpoint().velocity));
  }

  public void setAngle(int index) {
    currentAngle = index;
  }
}
