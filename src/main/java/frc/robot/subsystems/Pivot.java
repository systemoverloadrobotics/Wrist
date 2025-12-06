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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

  private final TalonFX pivotMotor; // Creating TalonFX motor

  // private final CANcoder pivotCANcoder; // Creating pivot encoder

  private final CANBus canBus; // Creates CANBus

  private final PositionVoltage pivotPosReq; // Creates Position Voltage Request

  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(Constants.Pivot.encoderChannel);

  public Pivot() {

    canBus = new CANBus("rio");

    pivotMotor = new TalonFX(Constants.Pivot.pivotId, canBus);

    pivotPosReq = new PositionVoltage(0);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.Pivot.kP;
    slot0Configs.kI = Constants.Pivot.kI;
    slot0Configs.kD = Constants.Pivot.kD;

    var MOCPivot = new MotorOutputConfigs();
    MOCPivot.Inverted = InvertedValue.CounterClockwise_Positive;
    MOCPivot.NeutralMode = NeutralModeValue.Brake;

    var feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    feedbackConfigs.SensorToMechanismRatio = Constants.Pivot.sensorToMechanismRatio;

    var currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.SupplyCurrentLimit = 30;

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0 = slot0Configs;
    pivotConfig.MotorOutput = MOCPivot;
    pivotConfig.Feedback = feedbackConfigs;
    pivotConfig.CurrentLimits = currentLimitsConfigs;
    pivotMotor.getConfigurator().apply(pivotConfig);

    // CANcoderConfiguration pivotCANcoderConfig = new CANcoderConfiguration(); // Creates wrist encoder configuration
    // pivotCANcoderConfig.MagnetSensor = new MagnetSensorConfigs()
    //     .withMagnetOffset(0.0);
    // pivotCANcoder.getConfigurator().apply(pivotCANcoderConfig);
    // Absolute encoder position -> internal encoder for wrist
    // pivotMotor.setPosition(pivotCANcoder.getAbsolutePosition().getValueAsDouble());
    pivotMotor.setPosition(pivotEncoder.get() + Constants.Pivot.pivotOfffset);
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
