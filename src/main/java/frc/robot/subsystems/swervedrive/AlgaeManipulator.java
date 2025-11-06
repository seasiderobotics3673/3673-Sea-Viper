// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.beans.Encoder;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.encoders.SparkFlexEncoderSwerve;

public class AlgaeManipulator extends SubsystemBase {
  private SparkFlex deployMotor = new SparkFlex(56, MotorType.kBrushless); 
  private SparkFlex beltMotor = new SparkFlex(57, MotorType.kBrushless); 
  private SparkFlexConfig deployMotorConfig = new SparkFlexConfig();
  private SparkFlexConfig beltMotorConfig = new SparkFlexConfig();

  private double deployMotorSpeed = 0.0;
  private double beltMotorSpeed = 0.0;

  private RelativeEncoder deployMotorEncoder = deployMotor.getEncoder();

  private int counter;
  

  /** Creates a new AlgaeManipulator. */
  public AlgaeManipulator() {
    deployMotor.configure(deployMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    beltMotor.configure(beltMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (deployMotorEncoder.getPosition() < Constants.ALGAE_MANIPULATOR_LOW_LIMIT) {
      if (deployMotorSpeed < 0) {
        setDeploySpeed(0);
      }
    } else {
      if (deployMotorEncoder.getPosition() > Constants.ALGAE_MANIPULATOR_HIGH_LIMIT) {
        if (deployMotorSpeed > 0) {
          setDeploySpeed(0);
        }
      }
    }

    //deployMotor.set(deployMotorSpeed);
    //beltMotor.set(beltMotorSpeed);
    //deployMotor.getAbsoluteEncoder().
    counter++;
    /*
    if (counter == 25) {
      counter = 0;
      System.out.println("deployMotor Encoder (Rotations) = " + deployMotorEncoder.getPosition());
    }
    */
  }

  public void setDeploySpeed(double deployMotorSpeed) {
    this.deployMotorSpeed = deployMotorSpeed;
    System.out.println("Deploy Motor Speed: " + deployMotorSpeed);
    deployMotor.set(deployMotorSpeed);
  }

  public void setBeltSpeed(double beltMotorSpeed) {
    this.beltMotorSpeed = beltMotorSpeed;
    System.out.println("Belt Motor Speed:" + beltMotorSpeed);
    beltMotor.set(beltMotorSpeed);
  }

}
