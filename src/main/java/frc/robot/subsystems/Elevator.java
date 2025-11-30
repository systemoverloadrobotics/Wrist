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
    private final TalonFX elevatorMotor1;

    private final TalonFX elevatorMotor2;

    // Absolute encoder
    private final CANcoder elevatorCANcoder;

    private final CANBus kCANBus = new CANBus("rio");

    private final PositionVoltage elevatorPosReq; 

    public Elevator() {
        elevatorMotor1 = new TalonFX(Constants.Elevator.motorId1);
        elevatorMotor2 = new TalonFX(Constants.Elevator.motorId2);
        elevatorCANcoder = new CANcoder(Constants.Elevator.CANcoderId, kCANBus);
        elevatorPosReq = new PositionVoltage(0); // Initializes wrist position request

        Slot0Configs slot0Configs = new Slot0Configs(); // Creates slot 0 configuration
        slot0Configs
                .withKP(5) // Adjusts time it takes to reach goal
                .withKI(0) // Fixes issue with kP not reaching goal because of equally opposing forces
                .withKD(0.5); // If overshoots because of kI, add kD

        // slot0Configs.kP = 10;
        var MOCElevator1 = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);

        // Motor Output Configuration for MOTOR 2 (follower, opposite inversion)
        var MOCElevator2 = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive); // Opposite of motor 1!

        // Configuration for Motor 1 (Leader)
        TalonFXConfiguration elevatorConfig1 = new TalonFXConfiguration();
        elevatorConfig1
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(5.583) // VERIFY THIS RATIO!
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Constants.Elevator.supplyCurrentLimit)) // FIXED!
                .withSlot0(slot0Configs)
                .withMotorOutput(MOCElevator1);

        // Configuration for Motor 2 (Follower)
        TalonFXConfiguration elevatorConfig2 = new TalonFXConfiguration();
        elevatorConfig2
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Constants.Elevator.supplyCurrentLimit))
                .withMotorOutput(MOCElevator2);

        elevatorMotor1.getConfigurator().apply(elevatorConfig1);
        elevatorMotor2.getConfigurator().apply(elevatorConfig2);

        // Set motor 2 to follow motor 1
        // elevatorMotor2.setControl(new Follower(Constants.Elevator.motorId1, false));

        CANcoderConfiguration elevatorCANcoderConfig = new CANcoderConfiguration(); // Creates elevator encoder
                                                                                    // configuration
        elevatorCANcoderConfig.MagnetSensor = new MagnetSensorConfigs().withMagnetOffset(0.0977777);
        elevatorCANcoder.getConfigurator().apply(elevatorCANcoderConfig);

        // Absolute encoder position -> internal encoder for elevator
        // Set initial position from absolute encoder
        double encodePos = elevatorCANcoder.getAbsolutePosition().getValueAsDouble();
        elevatorMotor1.setPosition(encodePos);
        elevatorMotor2.setPosition(encodePos);
    }

    public void setElevatorPosition(double position) {
        // Between -0.5 and 0.5 rotations; (?) --> should be between min and max height
        // Only control motor 1 - motor 2 follows automatically
        elevatorMotor1.setControl(elevatorPosReq.withPosition(position));
        elevatorMotor2.setControl(elevatorPosReq.withPosition(position));
    }

    public void setElevatorPosition(Angle position) {
        // Only control motor 1 - motor 2 follows automatically
        elevatorMotor1.setControl(elevatorPosReq.withPosition(position));
        elevatorMotor2.setControl(elevatorPosReq.withPosition(position));
    }
}