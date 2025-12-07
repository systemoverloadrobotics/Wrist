// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

private final TalonFX clawMotor; // Creating TalonFX motor
private final CANBus kCANBus = new CANBus("rio"); // Creates CANBus
private final DutyCycleOut clawPosReq; // Creates Position Voltage Request


  public Claw() {

    clawMotor = new TalonFX(Constants.Claw.motorId, kCANBus);
    clawPosReq = new DutyCycleOut(0);

    var MOCClaw = new MotorOutputConfigs() // Creates motor output configuration
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive);
    TalonFXConfiguration clawConfig = new TalonFXConfiguration() // Creates claw motor configuration                                                                     // claw
        .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.Claw.supplyCurrentLimit))
        .withMotorOutput(MOCClaw);
      
      clawMotor.getConfigurator().apply(clawConfig);

    
    }
    public void setSpeed(double speed) {
      clawMotor.setControl(clawPosReq.withOutput(speed));
      
    }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {

   
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
