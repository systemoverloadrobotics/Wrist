// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Wrist {
    public static final int wristId = 13;
    public static final int CANcoderId = 14;
    public static final double supplyCurrentLimit = 30.0;
    public static final double kP = 18;
    public static final double kI = 3.5;
    public static final double kD = 0.0;
    public static final double sensorToMechanismRatio = 5.583;
    public static final double CANCoderOffset = 0.0977777;
  }

  public static class Pivot {
    public static final int pivotId = 12;
    // public static final int CANcoderId = -1;
    public static final int encoderChannel = 5;
    public static final double pivotOfffset =  -0.4568;
    public static final double kP = 15;
    public static final double kI = 0;
    public static final double kD = 0.5;
    public static final double sensorToMechanismRatio = 30;
    public static final double supplyCurrentLimit = 40.0;
  }

  public static class Elevator {
    public static final int leftMotorId = 10;
    public static final int rightMotorId = 11;
    public static final double supplyCurrentLimit = 60.0;
    public static final double kP = 15;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double sensorToMechanismRatio = 12;
  }
}
