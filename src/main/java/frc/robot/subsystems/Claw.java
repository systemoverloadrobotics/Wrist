package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  private final TalonFX clawMotor; // Creates wrist motor

  // Absolute encoder
  // private final CANcoder clawCANcoder; // Creates wrist encoder

  private final CANBus kCANBus = new CANBus("rio");
  private final DutyCycleOut clawPosReq;

  public Claw() {
    clawMotor = new TalonFX(Constants.Wrist.wristId); // Initializes wrist motor
    // wristCANcoder = new CANcoder(Constants.Wrist.CANcoderId, kCANBus); //
    // Initializes wrist encoder
    clawPosReq = new DutyCycleOut(0); // Initializes wrist position request

    var MOCClaw = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    TalonFXConfiguration clawConfig = new TalonFXConfiguration() // Creates wrist motor configuration
        // wrist
        .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.Claw.supplyCurrentLimit))

        .withMotorOutput(MOCClaw);

    clawMotor.getConfigurator().apply(clawConfig);
  }

  public void setClawSpeed(double speed) {
    // Between -0.5 and 0.5 rotations; 1 rotation = 360 degrees
    // 1.5 rotations = 540 degrees (up)
    clawMotor.setControl(clawPosReq.withOutput(speed));
  }

}
