// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.GrabCoral;
import frc.robot.commands.OperatorDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final Elevator elevator = null;//new Elevator();
  public static final Intake intake = null;//new Intake();
  public static final Outtake outtake = null;//new Outtake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
      drivetrain.setDefaultCommand(new OperatorDrive(driverController));
      // Reset gyro and position
      new JoystickButton(driverController, XboxController.Button.kStart.value).onTrue(new InstantCommand() {
        @Override
        public void initialize() {
          drivetrain.resetGyro(0);
          drivetrain.resetPosition(0, 0);
        }
      });
      /*// Intake control
      new Trigger(() -> driverController.getRightTriggerAxis() > 0.75).whileTrue(new Command () {
        @Override
        public void initialize() {
          intake.setIntakeOn();
        };

        @Override
        public void end(boolean interrupted) {
          intake.setIntakeOff();
        };
      });
      // Elevator level control (starting with bottom letter button for lowest level and going clockwise)
      new JoystickButton(driverController, XboxController.Button.kA.value).onTrue(new InstantCommand() {
        @Override
        public void initialize() {
          elevator.setLevel(0);
          outtake.setIndexedAngle(0);
        }
      });
      //No level 1 for now
      new JoystickButton(driverController, XboxController.Button.kX.value).onTrue(new InstantCommand() {
        @Override
        public void initialize() {
          elevator.setLevel(2);
          outtake.setIndexedAngle(1);
        }
      });
      new JoystickButton(driverController, XboxController.Button.kY.value).onTrue(new InstantCommand() {
        @Override
        public void initialize() {
          elevator.setLevel(3);
          outtake.setIndexedAngle(1);
        }
      });
      new JoystickButton(driverController, XboxController.Button.kB.value).onTrue(new InstantCommand() {
        @Override
        public void initialize() {
          elevator.setLevel(4);
          outtake.setIndexedAngle(2);
        }
      });
      // Grab coral manually or when coral notifier triggers
      new JoystickButton(driverController, XboxController.Button.kLeftBumper.value).onTrue(new GrabCoral());
      new Trigger(() -> new DigitalInput(OperatorConstants.coralNotifierID).get()).onTrue(new GrabCoral());
      // Drop coral
      new JoystickButton(driverController, XboxController.Button.kRightBumper.value).whileTrue(outtake.dropCoral());*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous (nothing)
    return new InstantCommand();
  }
}
