// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  

  private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorId, MotorType.kBrushless);

  public Intake() {
    intakeMotor.setIdleMode(IdleMode.kCoast);
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

  public void setIntakeCustom(double amount) {
    intakeMotor.set(amount);
  }

  // Stop intaking
  public void setIntakeOff() {
    intakeMotor.set(0);
  }
}
