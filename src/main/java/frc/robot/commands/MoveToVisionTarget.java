// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import static edu.wpi.first.units.Units.Celsius;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToVisionTarget extends Command {

  private Vision vision;
  private SwerveSubsystem driveBase;
  private int counter;
  /** Creates a new MoveToVisionTarget. */
  public MoveToVisionTarget(Vision vision, SwerveSubsystem driveBase) {
    this.vision = vision;
    this.driveBase = driveBase;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<EstimatedRobotPose> result = vision.getEstimatedGlobalPose(Cameras.CENTER_CAM);
    System.out.println("Got latest result from vision." + counter);
    counter++;
  




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter >= 20) {
      return true;
    }
    return false;
  }
}
