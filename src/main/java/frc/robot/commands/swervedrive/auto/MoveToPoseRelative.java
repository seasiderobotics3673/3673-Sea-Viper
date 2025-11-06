// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToPoseRelative extends Command {
  private double deltaX;
  private double deltaY;
  private double deltaRot;
  private SwerveSubsystem drivebase;
  private int duration; // time in miliseconds

  private int totalSteps;
  private int currentStep;

  private boolean isXFinished;
  private boolean isYFinished;
  private boolean isRotFinished;
  private Pose2d startingPose;

  private double startX;
  private double startY;
  private double startRot;

  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;

  //private final double defaultXSpeed = 3.0;
  //private final double defaultYSpeed = 3.0;
  
  private double destX;
  private double destY;
  private double destRot;

  private double lastRot;

  private int dirX;
  private int dirY;
  private int dirRot;

  public int counter;

  public int offsetRot;

  public boolean isXFinishedFlag;
  public boolean isYFinishedFlag;
  public boolean isRotFinishedFlag;

  

  /** Creates a new MoveToPoseRelative. */
  public MoveToPoseRelative(double deltaX, double deltaY, double deltaRot, int duration, SwerveSubsystem drivebase) {
    this.deltaX = deltaX;
    this.deltaY = deltaY;
    this.deltaRot = deltaRot;
    this.drivebase = drivebase;
    this.duration = duration;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    xSpeed = 3.0;
    ySpeed = 3.0;
    rotSpeed = 2.0;


    totalSteps = duration / 20; // 20ms is a default cycle time
    currentStep = 0;
    startingPose = drivebase.getPose();

    startX = startingPose.getX();
    startY = startingPose.getY();
    startRot = startingPose.getRotation().getDegrees();

    lastRot = startRot;

    destX = startX - deltaX;
    destY = startY - deltaY;
    destRot = startRot + deltaRot;

    if (destRot > 180) {
      offsetRot = -360;
      destRot -= 360;
    } else {
      if (destRot < -180) {
        offsetRot = 360;
        destRot += 360;
      }
    }

    dirX = (int) Math.signum(deltaX);
    dirY = (int) Math.signum(deltaY);
    dirRot = (int) Math.signum(destRot - (startRot + offsetRot));

    xSpeed *= dirX;
    ySpeed *= dirY;
    rotSpeed *= dirRot;
    counter = 0;
    isXFinishedFlag = false;
    isYFinishedFlag = false;
    isRotFinishedFlag = false;

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.drive(new Translation2d(-xSpeed, -ySpeed), -rotSpeed, true);
    counter++;
    if (counter >= 50) {
      counter = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(0, 0), 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isXFinishedFlag = isXFinished();
    boolean isYFinishedFlag = isYFinished();
    boolean isRotFinishedFlag = isRotFinished();
    return isXFinishedFlag && isYFinishedFlag && isRotFinishedFlag;
  }

  public boolean isXFinished() {
    double currX = drivebase.getPose().getX();
    if (counter == 1) {
      System.out.print("dirX = " + dirX);
      System.out.print(" currX = " + currX);
      System.out.println(" destX = " + destX);
    }

    switch(dirX) {
      case 1:
        if (currX < destX) {
          xSpeed = 0;
          return true;
        }
        break;
      case -1:
        if (currX > destX) {
          xSpeed = 0;
          return true;
        }
        break;
      case 0:
        return true;
    }
    return false;
  }



  public boolean isYFinished() {
    double currY = drivebase.getPose().getY();
    if (counter == 1) {
      System.out.print("dirY = " + dirY);
      System.out.print("currY = " + currY);
      System.out.println("destY = " + destY);
    }
    switch(dirY) {
      case 1:
        if (currY < destY) {
          ySpeed = 0;
          return true;
        }
        break;
      case -1:
        if (currY > destY) {
          ySpeed = 0;
          return true;
        }
        break;
      case 0:
        return true;
    }
    return false;
  }
  public boolean isRotFinished() {
    double currRot = drivebase.getPose().getRotation().getDegrees();
    if (dirRot > 0 && currRot < lastRot) {
      offsetRot = 0;
    }
    else {
      if (dirRot < 0 && currRot > lastRot) {
        offsetRot = 0;
      }
    }
    currRot += offsetRot;
    if (counter == 1) {
      System.out.print("dirRot = " + dirRot + "   ");
      System.out.print("currRot = " + currRot + "   ");
      //System.out.print("startingRot = " + startingPose.getRotation().getDegrees());
      System.out.println("destRot = " + destRot);
    }
    switch(dirRot) {
      case 1:
        if (currRot > destRot) {
          rotSpeed = 0;
          return true;
        }
        break;
      case -1:
        if (currRot < destRot) {
          rotSpeed = 0;
          return true;
        }
        break;
      case 0:
        return true;
    }
    return false;
  }
}
