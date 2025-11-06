// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateAbsolute extends Command {
  /** Creates a new RotateAbsolute. */

  private SwerveSubsystem driveBase;
  private double curHeading;
  private double destHeading;
  private double rotSpeed;
  private int counter = 0;

  public RotateAbsolute(double destHeading, SwerveSubsystem driveBase) {
    this.driveBase = driveBase;
    this.destHeading = destHeading;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBase);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    curHeading = driveBase.getPose().getRotation().getDegrees();
    if (curHeading > destHeading) {
      rotSpeed = -2.0;
    } else {
      rotSpeed = 2.0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBase.drive(new Translation2d(0, 0), -rotSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.drive(new Translation2d(0, 0), 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    curHeading = driveBase.getPose().getRotation().getDegrees();
    if (counter == 0) {
      System.out.print("CurHeading = " + curHeading);
      System.out.print("RotSpeed = " + rotSpeed);
      System.out.println("DestHeading = " + destHeading);
    }

    counter++;
    if (counter > 25) {
      counter = 0;
    }
    if (rotSpeed > 0.0) {
      if (curHeading > destHeading) {
        System.out.println("Stopped moving forward towards dest");
        return true;
      }
    } else {
      if (rotSpeed < 0.0) {
        if (curHeading < destHeading) {
          System.out.println("Stopped moving backwards towards dest");
          return true;
        }
      }
      
    }
    return false;
  }
}
