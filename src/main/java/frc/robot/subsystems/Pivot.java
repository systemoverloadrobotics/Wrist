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

public class Pivot extends SubsystemBase {

  private final TalonFX pivotMotor; // Creating TalonFX motor

  private final CANcoder pivotCANcoder; // Creating pivot encoder

  private final CANBus canBus; // Creates CANBus

  private final PositionVoltage pivotPosReq; // Creates Position Voltage Request

  public Pivot() {

    canBus = new CANBus("rio");

    pivotMotor = new TalonFX(Constants.Pivot.pivotId, canBus);
    pivotCANcoder = new CANcoder(Constants.Pivot.CANcoderId, canBus);

    pivotPosReq = new PositionVoltage(0);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 10;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    var MOCPivot = new MotorOutputConfigs();
    MOCPivot.Inverted = InvertedValue.CounterClockwise_Positive;
    MOCPivot.NeutralMode = NeutralModeValue.Brake;

    var feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    feedbackConfigs.SensorToMechanismRatio = 1.0;

    var currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.SupplyCurrentLimit = 30;

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0 = slot0Configs;
    pivotConfig.MotorOutput = MOCPivot;
    pivotConfig.Feedback = feedbackConfigs;
    pivotConfig.CurrentLimits = currentLimitsConfigs;
    pivotMotor.getConfigurator().apply(pivotConfig);

    CANcoderConfiguration pivotCANcoderConfig = new CANcoderConfiguration(); // Creates wrist encoder configuration
    pivotCANcoderConfig.MagnetSensor = new MagnetSensorConfigs()
        .withMagnetOffset(0.0);
    pivotCANcoder.getConfigurator().apply(pivotCANcoderConfig);
    // Absolute encoder position -> internal encoder for wrist
    pivotMotor.setPosition(pivotCANcoder.getAbsolutePosition().getValueAsDouble());
  }
    
  public void setPivotPosition(double position) {
    // Between -0.5 and 0.5 rotations; 1 rotation = 360 degrees
    // 1.5 rotations = 540 degrees (up)
    pivotMotor.setControl(pivotPosReq.withPosition(position));
  }

  public void setPivotPosition(Angle position) {
    // Uses angle measurements instead
    pivotMotor.setControl(pivotPosReq.withPosition(position));
  }
}
