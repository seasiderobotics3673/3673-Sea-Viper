// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  
  private SparkMax leftMotor = new SparkMax(51, MotorType.kBrushless);
  private SparkMax rightMotor = new SparkMax(52, MotorType.kBrushless);
  private Encoder elevatorEncoder = new Encoder(Constants.ELEVATOR_ENCODER_A_DIO, Constants.ELEVATOR_ENCODER_B_DIO);

  private SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

  private SparkLimitSwitch highLimitSwitch = leftMotor.getForwardLimitSwitch();
  private SparkLimitSwitch lowLimitSwitch = leftMotor.getReverseLimitSwitch();

  private int maxEncoderValue = 8639;
  private double maxHeightInInches = 65.5;

  private int minEncoderValue = 0;
  private double minHeightInInches = 18.25;

  private int counter;


  public final double stopsInInches [] = {
    21.0,//topOfLowerPlate-EncoderReading:123-111
    31.0,//-EncoderReading:2590-13.375inchDifFromPrev-2472EncoderDif
    46.675,//EncoderReading:5291,5517-15.75inchDifFromPrev-2927EncoderDif
    64.375,//Can't reach!
    18.75//LoadFromHopper-EncoderReading:?
    //64.375? Max Height-EncoderReading:8815-16.7inchDifFromL3-3524EncoderDif
  };
  //L3 - 
  public final int L1 = 0;
  public final int L2 = 1;
  public final int L3 = 2;
  public final int L4 = 3;
  public final int HOPPER = 4;

  private double destinationInTicks = 0.0;

  private double motorSpeed = Constants.ELEVATOR_ANTIGRAVITY;
  
    /** Creates a new Elevator. */
    public Elevator() {
      leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  

      rightMotorConfig.follow(leftMotor, true);
      rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      counter = 0;
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      counter++;

      if (getHeightInInches() > stopsInInches[L3] + 1.0 && Math.signum(motorSpeed) == 1) {
        leftMotor.set(Constants.ELEVATOR_ANTIGRAVITY); 
        if (counter >= 50){
          System.out.println("softLimitAt = " + getHeightInInches());
        }
      } else {
        leftMotor.set(motorSpeed);
      }

      if (counter >= 50) {
        counter = 0;
        //printEncoderValue();
      }
      
    }
    public int getHeightInTicks() {
      return elevatorEncoder.get() - minEncoderValue;
    }

    public double getHeightInInches() {
      return (elevatorEncoder.get() - minEncoderValue) / Constants.ELEVATOR_TICKS_PER_INCH + minHeightInInches;
    }
  
    public void setDestinationInInches(double destinationInInches) {
      this.destinationInTicks = (destinationInInches - minHeightInInches) * Constants.ELEVATOR_TICKS_PER_INCH + minEncoderValue;
      System.out.println("destinationInInches = " + destinationInInches);
      System.out.println("destinationInTicks = " + destinationInTicks);
    }
  
    public double getDestinationInTicks() {
      return destinationInTicks;
    }

    
  
    public void goToPreset(int preset) {
      setDestinationInInches(stopsInInches[preset]);
    }
    
    public void setMotorSpeed(double motorSpeed) {
      this.motorSpeed = motorSpeed;
  }
    public void changeMotorSpeed(double deltaMotorSpeed) {
      this.motorSpeed += deltaMotorSpeed;
      System.out.println("Elevator speed = " + motorSpeed);
    }
    
    public boolean isAtHighLimit() {
      return highLimitSwitch.isPressed();
    }

    public boolean isAtLowLimit() {
      return lowLimitSwitch.isPressed();
    }
    
    public void setMaxEncoderValue() {
      maxEncoderValue = elevatorEncoder.get();
      System.out.println("High Limit Switch Hit; New Number: " + maxEncoderValue);
    }

    public void setMinEncoderValue() {
      //minEncoderValue = elevatorEncoder.get();
      maxEncoderValue -= elevatorEncoder.get();
      elevatorEncoder.reset();
      System.out.println("Low Limit Switch Hit");
    }
    public void printEncoderValue() {
      System.out.println("ElevatorEncoder.get() = " + elevatorEncoder.get());
    }
    public int inchesToTicks(double inches) {
      double ticksPerInch = (maxEncoderValue - minEncoderValue) / (maxHeightInInches - minHeightInInches);
      return (int) ((inches - minHeightInInches) * ticksPerInch + minEncoderValue); 
      //return -1; // if math does not work
    }
}
