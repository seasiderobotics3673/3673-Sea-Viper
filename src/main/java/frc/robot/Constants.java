// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.
  public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second; 2-5-2025

  // 3673 constants here
  public static final int ELEVATOR_ENCODER_A_DIO = 2;
  public static final int ELEVATOR_ENCODER_B_DIO = 3;
  public static final double ELEVATOR_TICKS_PER_INCH = 184.8;
  public static final double ELEVATOR_TOLERANCE = 30.8; // this is 1/6th of an inch
  public static final double ELEVATOR_RAMPDOWN = 659.45; //In Ticks. Roughly 5 Inches
  public static final double ELEVATOR_SPEED_UP = 0.40;
  public static final double ELEVATOR_SPEED_DOWN = 0.30;
  public static final double ELEVATOR_SPEED_UP_SLOW = 0.13;
  public static final double ELEVATOR_SPEED_DOWN_SLOW = 0.11;
  public static final double ELEVATOR_MANUAL_SPEED_UP = 0.15;
  public static final double ELEVATOR_MANUAL_SPEED_UP_INVERSE = -0.15;
  public static final double ELEVATOR_MANUAL_SPEED_DOWN = -0.15;
  public static final double ELEVATOR_MANUAL_SPEED_DOWN_INVERSE = 0.15;

  //public static final int FEEDER_CORAL_PROXIMITY_RANGE = 80;
  public static final int FEEDER_CORAL_PROXIMITY_HIGHBOUND = 180;
  public static final int FEEDER_CORAL_PROXIMITY_LOWBOUND = 100;

  public static final double ELEVATOR_ANTIGRAVITY = 0.03;
  //public static final double ELEVATOR_ANTIGRAVITY = 0.0; // disabled for testing, reenable later

  public static final double ALGAE_MANIPULATOR_HIGH_LIMIT = 0.25;
  public static final double ALGAE_MANIPULATOR_LOW_LIMIT = 0;

  public static final double SWERVE_SPEED_FULL = 1.0;
  public static final double SWERVE_SPEED_SLOW = 0.5;

  public static final double VISION_TURN_kP = 0.01;



//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
