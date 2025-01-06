// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  CANSparkMax motor = new CANSparkMax(ElevatorConstants.motorID, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();
  DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.limitSwitchID);

  ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(ElevatorConstants.maxVel, ElevatorConstants.maxAccel);
  ProfiledPIDController controller = new ProfiledPIDController(ElevatorConstants.p, ElevatorConstants.i, ElevatorConstants.d, constraints);

  int currentLevel = 0;

  public Elevator() {
    motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) {
      moveToSetpoint(ElevatorConstants.levels[currentLevel]);
    }
    
    if (limitSwitch.get()) {
      encoder.setPosition(0);
    }
  }

  double getPosition() {
    return encoder.getPosition() * ElevatorConstants.positionConversionFactor;
  }

  /*
   * 
   */
  public void moveToSetpoint(double setpoint) {
    motor.setVoltage(
      controller.calculate(getPosition())
        + feedforward.calculate(controller.getSetpoint().velocity));
  }

  /**
   * Sets the target level of the elevator by index (0 for base, 1 for reef trough, 4 for reef top)
   */
  public void setLevel(int index) {
    currentLevel = index;
  }

}

//comment
