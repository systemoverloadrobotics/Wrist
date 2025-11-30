package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {

  private final TalonFX wristMotor; // Creates wrist motor

  // Absolute encoder
  private final CANcoder wristCANcoder; // Creates wrist encoder

  private final CANBus kCANBus = new CANBus("rio");
  private final PositionVoltage wristPosReq;

  public Wrist() {
    wristMotor = new TalonFX(Constants.Wrist.motorId); // Initializes wrist motor
    wristCANcoder = new CANcoder(Constants.Wrist.CANcoderId, kCANBus); // Initializes wrist encoder
    wristPosReq = new PositionVoltage(0); // Initializes wrist position request

    Slot0Configs slot0Configs = new Slot0Configs() // Creates slot 0 configuration
        .withKP(18) // Adjusts time it takes to reach goal
        .withKI(3.5) // Fixes issue with kP not reaching goal because of equally opposing forces
        .withKD(0); // If overshoots because of kI, add kD

    var MOCWrist = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    TalonFXConfiguration wristConfig = new TalonFXConfiguration() // Creates wrist motor configuration
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(5.583)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))// ratio between motor and
                                                                             // wrist
        .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.Wrist.supplyCurrentLimit))
        .withSlot0(slot0Configs)
        .withMotorOutput(MOCWrist);

    wristMotor.getConfigurator().apply(wristConfig);

    CANcoderConfiguration wristCANcoderConfig = new CANcoderConfiguration(); // Creates wrist encoder configuration
    wristCANcoderConfig.MagnetSensor = new MagnetSensorConfigs()
        .withMagnetOffset(0.0977777);
    wristCANcoder.getConfigurator().apply(wristCANcoderConfig);
    // Absolute encoder position -> internal encoder for wrist
    wristMotor.setPosition(wristCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  public void setWristPosition(double position) {
    // Between -0.5 and 0.5 rotations; 1 rotation = 360 degrees
    // 1.5 rotations = 540 degrees (up)
    wristMotor.setControl(wristPosReq.withPosition(position));
  }

  public void setWristPosition(Angle position) {
    // Uses angle measurements instead
    wristMotor.setControl(wristPosReq.withPosition(position));
  }
}
