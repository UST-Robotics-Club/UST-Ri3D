// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {
  public Outtake() {}

  SparkMax arm = new SparkMax(0, null);
  SparkMax extension = new SparkMax(0, null);
  SparkMax claw = new SparkMax(0, null);

  @Override
  public void periodic() {
    
  }

}
