// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

/*
 * Intake, Hold
 * L1, L2, L3, L4
 * HP, Barge, Processor
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Wrist wrist = new Wrist();
  private final Pivot pivot = new Pivot();
  private final Elevator elevator = new Elevator();

  boolean isIntaking = false;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController joystick =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // onTrue = key down
    // onFalse = key up
    // whileTrue = key held
    // whileFalse = key not held

    // joystick.b().onTrue(Commands.runOnce(() -> {
    //   wrist.setWristPosition(0.25); // Set to 90 degrees from the origin
    // }));

    // joystick.a().onTrue(Commands.runOnce(() -> {
    //   wrist.setWristPosition(0.0); // Set to 0 degrees from the origin
    // }));


    joystick.a().whileTrue(Commands.runOnce(RobotContainer.this::goToIntake));

    joystick.a().whileFalse(Commands.runOnce(RobotContainer.this::goToHold));

    joystick.b().onTrue(Commands.runOnce(RobotContainer.this::scoreL1));
    joystick.b().onFalse(Commands.runOnce(RobotContainer.this::scoreL1));

    joystick.x().onTrue(Commands.runOnce(RobotContainer.this::scoreL2));
    joystick.x().onFalse(Commands.runOnce(RobotContainer.this::scoreL2));

    joystick.a().onTrue(Commands.runOnce(() -> {
      isIntaking = true;
    }));

   

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public void goToIntake() {
    if (isIntaking) {
      wrist.setWristPosition(0);
      pivot.setPivotPosition(Degrees.of(0));
      elevator.setElevatorPosition(0.2);
    }
  }

  public void goToHold() {
    if (isIntaking) {
      wrist.setWristPosition(0);
      pivot.setPivotPosition(Degrees.of(75));
      elevator.setElevatorPosition(0.2);
    }
  }
  public void scoreL1(){
    wrist.setWristPosition(0);
    pivot.setPivotPosition(Degrees.of(50));
    elevator.setElevatorPosition(0.3);
  }
  public void scoreL2(){
    wrist.setWristPosition(90);
    pivot.setPivotPosition(Degrees.of(75));
    elevator.setElevatorPosition(1);
  }
}

