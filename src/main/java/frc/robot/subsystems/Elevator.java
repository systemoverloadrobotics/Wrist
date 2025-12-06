package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    // create elevator motors that move the elevator up and down in sync
    private final TalonFX leftMotor;

    private final TalonFX rightMotor;

    // Absolute encoder
    private final CANBus kCANBus = new CANBus("rio");

    private final PositionVoltage elevatorPosReq; 

    public Elevator() {
        leftMotor = new TalonFX(Constants.Elevator.leftMotorId, kCANBus);
        rightMotor = new TalonFX(Constants.Elevator.rightMotorId, kCANBus);
        elevatorPosReq = new PositionVoltage(0); // Initializes wrist position request

        Slot0Configs slot0Configs = new Slot0Configs(); // Creates slot 0 configuration
        slot0Configs
                .withKP(Constants.Elevator.kP) // Adjusts time it takes to reach goal
                .withKI(Constants.Elevator.kI) // Fixes issue with kP not reaching goal because of equally opposing forces
                .withKD(Constants.Elevator.kD); // If overshoots because of kI, add kD

        // slot0Configs.kP = 10;
        var leftMotorMOC = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);

        // Motor Output Configuration for MOTOR 2 (follower, opposite inversion)
        var rightMotorMOC = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive); // Opposite of motor 1!

        // Configuration for Motor 1 (Leader)
        TalonFXConfiguration leftElevatorConfig = new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(Constants.Elevator.sensorToMechanismRatio)
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Constants.Elevator.supplyCurrentLimit))
                .withSlot0(slot0Configs)
                .withMotorOutput(leftMotorMOC);

        // Configuration for Motor 2 (Follower)
        TalonFXConfiguration rightElevatorConfig = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Constants.Elevator.supplyCurrentLimit))
                .withMotorOutput(rightMotorMOC);

        leftMotor.getConfigurator().apply(leftElevatorConfig);
        rightMotor.getConfigurator().apply(rightElevatorConfig);

        // Set motor 2 to follow motor 1
        // elevatorMotor2.setControl(new Follower(Constants.Elevator.motorId1, false));

        // Absolute encoder position -> internal encoder for elevator
        // Set initial position from absolute encoder
        leftMotor.setPosition(0);
        rightMotor.setPosition(leftMotor.getPosition().getValueAsDouble());
    }

    public void setElevatorPosition(double position) {
        // Between -0.5 and 0.5 rotations; (?) --> should be between min and max height
        // Only control motor 1 - motor 2 follows automatically
        leftMotor.setControl(elevatorPosReq.withPosition(position));
        rightMotor.setControl(elevatorPosReq.withPosition(position));
    }

    public void setElevatorPosition(Angle position) {
        // Only control motor 1 - motor 2 follows automatically
        leftMotor.setControl(elevatorPosReq.withPosition(position));
        rightMotor.setControl(elevatorPosReq.withPosition(position));
    }
}