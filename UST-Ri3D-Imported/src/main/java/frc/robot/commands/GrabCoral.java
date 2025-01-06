// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class GrabCoral extends Command {
  boolean isClawPhase = false;
  long timeToEnd = 0;

  public GrabCoral() {
    addRequirements(RobotContainer.elevator, RobotContainer.outtake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.outtake.setIndexedAngle(0);
    RobotContainer.elevator.setLevel(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!isClawPhase && RobotContainer.elevator.isFullyRetracted()) {
      RobotContainer.outtake.setClaw(0.5);
      timeToEnd = System.currentTimeMillis() + 1000;
      isClawPhase = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.outtake.setClaw(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isClawPhase && System.currentTimeMillis() >= timeToEnd;
  }
}
