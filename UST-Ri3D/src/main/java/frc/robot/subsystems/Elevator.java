// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  SparkMax motor = new SparkMax(ElevatorConstants.elevatorID, MotorType.kBrushless);

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
  }

  /**
   * Sets the target level of the elevator by index (0 for base, 3 for top)
   */
  private void setLevel(int index) {
    moveMeters(ElevatorConstants.levels[index] - ElevatorConstants.levels[currentLevel]);
    currentLevel = index;
  }

  private void moveMeters(double meters) {
    double deltaRotations = meters / ElevatorConstants.positionConversionFactor;
    motor.getClosedLoopController().setReference(motor.getEncoder().getPosition() + deltaRotations, ControlType.kPosition);
  }

}

//comment