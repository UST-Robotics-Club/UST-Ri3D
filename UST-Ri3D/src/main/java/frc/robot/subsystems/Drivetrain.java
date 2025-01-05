// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  SparkMax frontLeft = new SparkMax(1, MotorType.kBrushless);
  SparkMax frontRight = new SparkMax(2, MotorType.kBrushless);
  SparkMax backLeft = new SparkMax(3, MotorType.kBrushless);
  SparkMax backRight = new SparkMax(4, MotorType.kBrushless);
  public Drivetrain() {

  }

  // Robot relative. X = forward, y = left, rot = counterclockise
  public void drive(double x, double y, double rot) {
    
  }
  @Override
  public void periodic() {
    
  }

}
