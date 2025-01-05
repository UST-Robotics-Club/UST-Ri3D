// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class OperatorDrive extends Command {

  XboxController controller;
  public OperatorDrive(XboxController controller) {
    this.controller = controller;
    addRequirements(RobotContainer.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = controller.getLeftY();
    double left = controller.getLeftX();
    double rot = controller.getRightX();
    ChassisSpeeds speeds = new ChassisSpeeds(forward, left, rot);
    RobotContainer.drivetrain.drivePercent(speeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
