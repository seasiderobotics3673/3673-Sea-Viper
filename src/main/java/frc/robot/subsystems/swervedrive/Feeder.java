// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import java.lang.reflect.Array;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;



import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private SparkFlex leftMotor = new SparkFlex(61, MotorType.kBrushless);
  private SparkFlex rightMotor = new SparkFlex(62, MotorType.kBrushless);

  private SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
  private SparkFlexConfig rightMotorConfig = new SparkFlexConfig();

  private double leftMotorSpeed = 0.0;
  private double rightMotorSpeed = 0.0;

  private final I2C.Port i2cPort = I2C.Port.kMXP;

  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  private int proximity;

  private boolean wasCoralPrevPresent;

  private int counter;

  //private int proxSampleList[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  private int proxSampleList[] = {0};

  private int proxSampleLRU = 0; //LRU means Least Recently Used

  private int proxSampleTotal;

  private int proxSampleCount;

  //private final ColorMatch colorMatcher = new ColorMatch(); 
  /** Creates a new Feeder. */
  public Feeder() {
    rightMotorConfig.inverted(true);
    leftMotorConfig.inverted(false);
    
    leftMotor.configure(leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    wasCoralPrevPresent = false;



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftMotor.set(leftMotorSpeed);
    rightMotor.set(rightMotorSpeed);
    proximity = colorSensor.getProximity();
    //proximity = averageOfProx();
    //System.out.println("feeder.periodic()");
    counter++;
    if (counter >= 20) {
      counter = 0;
      if (averageOfProx() != 0) {
        //System.out.println("Avg Proximity from sensor: " + averageOfProx());
      }
    }
    proxSampleTotal -= proxSampleList[proxSampleLRU];
    proxSampleList[proxSampleLRU] = proximity;
    proxSampleTotal += proximity;
    proxSampleLRU++;
    if (proxSampleLRU >= proxSampleList.length) {
      proxSampleLRU = 0;
    }
    
    if (proxSampleCount < proxSampleList.length) {
      proxSampleCount++;
    }

  }

  public void setSpeeds(double leftMotorSpeed, double rightMotorSpeed) {
    this.leftMotorSpeed = leftMotorSpeed;
    this.rightMotorSpeed = rightMotorSpeed;
    System.out.println("Left Feeder Speed: " + leftMotorSpeed + " Right Feeder Speed: " + rightMotorSpeed);
    leftMotor.set(leftMotorSpeed);
    rightMotor.set(rightMotorSpeed);
  }

  public boolean isCoralPresent() {
    if (averageOfProx() >= Constants.FEEDER_CORAL_PROXIMITY_HIGHBOUND) {
      return true;
    } else {
      if (averageOfProx() <= Constants.FEEDER_CORAL_PROXIMITY_LOWBOUND) {
        return false;
      } else {
        return wasCoralPrevPresent;
      }
    }
  }

  public boolean hasCoralDisappeared() {
    if (isCoralPresent() == wasCoralPrevPresent) {
      return false;
    } else {
      wasCoralPrevPresent = isCoralPresent();
      if (isCoralPresent() == false) {
        System.out.println("State Change: Coral Not Present");
        return true;
      } else {
        System.out.println("State Change: Coral Detected");
        return false;
      }
    }
  }

  public int averageOfProx() {
    return (proxSampleCount == 0) ? 0 : proxSampleTotal / proxSampleCount;
  }
}
