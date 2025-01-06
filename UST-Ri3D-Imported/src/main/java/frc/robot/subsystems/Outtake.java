// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

public class Outtake extends SubsystemBase {
  

  CANSparkMax arm = new CANSparkMax(OuttakeConstants.armMotorId, MotorType.kBrushless);
  CANSparkMax claw = new CANSparkMax(OuttakeConstants.clawMotorId, MotorType.kBrushless);
  RelativeEncoder armEncoder = arm.getEncoder();
  
  ArmFeedforward feedforwardArm = new ArmFeedforward(OuttakeConstants.kS, OuttakeConstants.kG, OuttakeConstants.kV);
  TrapezoidProfile.Constraints constraintsArm = new TrapezoidProfile.Constraints(OuttakeConstants.maxVel, OuttakeConstants.maxAccel);
  ProfiledPIDController controllerArm = new ProfiledPIDController(OuttakeConstants.pArm, OuttakeConstants.iArm, OuttakeConstants.dArm, constraintsArm);
  int currentAngle = 0;
  public Outtake() {
    arm.setIdleMode(IdleMode.kBrake);
    claw.setIdleMode(IdleMode.kBrake);
  }
  @Override
  public void periodic() {
    //arm.getEncoder()
    if (DriverStation.isEnabled()) {
      angleArmToSetpoint(OuttakeConstants.angles[currentAngle]);
    }
  }

  double getArmDegrees() {
    return armEncoder.getPosition() * OuttakeConstants.angleConversionFactor;
  }

  double getArmRadians() {
    return Math.toRadians(getArmDegrees());
  }

  public void setClaw(double speed) {
    claw.set(speed);
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
   * Setpoint in degrees. 0 = down
   */
  public void angleArmToSetpoint(double setpoint) {
    arm.setVoltage(
      controllerArm.calculate(getArmDegrees(), setpoint)
        + feedforwardArm.calculate(getArmRadians(), controllerArm.getSetpoint().velocity));
  }

  public void setIndexedAngle(int index) {
    currentAngle = index;
  }
}
