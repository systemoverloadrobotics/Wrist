// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;







public class Claw extends SubsystemBase{
    private final TalonFX clawMotor;

    private final CANBus kCANBus = new CANBus("rio");

    private final DutyCycleOut clawDutyCycleOut;

    public Claw(){
        clawMotor = new TalonFX(Constants.Claw.clawMotorId, kCANBus);
        clawDutyCycleOut = new DutyCycleOut(0);


        var clawMotorMOC = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive);
        var clawConfig = new TalonFXConfiguration();
        clawConfig.MotorOutput = clawMotorMOC; // sets motor output to clawConfig object/var

        clawMotor.getConfigurator().apply(clawConfig);
    }

    public void intake() {
        clawMotor.setControl(clawDutyCycleOut.withOutput(0.5));
    }

    public void outtake() {
        clawMotor.setControl(clawDutyCycleOut.withOutput(-0.2));
    }
    
}
