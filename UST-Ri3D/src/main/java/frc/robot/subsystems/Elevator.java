// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  SparkMax motor = new SparkMax(ElevatorConstants.motorID, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();
  DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.limitSwitchID);

  ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0); // TODO: Find feedforward constants using SysID
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0, 0); // TODO: Find trapezoid profile constraints
  ProfiledPIDController controller = new ProfiledPIDController(ElevatorConstants.p, ElevatorConstants.i, ElevatorConstants.d, constraints);

  int currentLevel = 0;

  public Elevator() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.closedLoop
      .p(ElevatorConstants.p)
      .i(ElevatorConstants.i)
      .d(ElevatorConstants.d)
      .outputRange(ElevatorConstants.minOutput, ElevatorConstants.maxOutput);
    motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
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
