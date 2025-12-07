// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
    private final TalonFX clawMotor; // Creates claw motor
    private final DutyCycleOut dutyCycleReq = new DutyCycleOut(0);

  /** Creates a new ExampleSubsystem. */
  public Claw() {
    clawMotor = new TalonFX(Constants.Claw.motorId); // Initializes claw motor
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void intake() {
    this.setPower(0.5);
  }

  public void outtake() {
    this.setPower(-0.5);
  }

  public void stop() {
    this.setPower(0);
  }

  public void setPower(double power) {
    // Between -0.5 and 0.5 rotations; 1 rotation = 360 degrees
    // 1.5 rotations = 540 degrees (up)
    clawMotor.setControl(dutyCycleReq.withOutput(power));
  }
}
