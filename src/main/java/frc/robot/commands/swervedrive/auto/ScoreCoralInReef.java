// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveElevatorToLevel;
import frc.robot.commands.RotateAbsolute;
import frc.robot.subsystems.swervedrive.AlgaeManipulator;
import frc.robot.subsystems.swervedrive.Elevator;
import frc.robot.subsystems.swervedrive.Feeder;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCoralInReef extends SequentialCommandGroup {
  /** Creates a new ScoreCoralInReef. */
  public ScoreCoralInReef(double startingYaw, SwerveSubsystem drivebase, Feeder feeder, Elevator elevator, AlgaeManipulator algaeManipulator) {
    // Add your commands in the addCommands() call, e.g.) {
    //Below section does not account for when up against the coral, aka when the limit switch is pressed.
    addCommands(
      new InstantCommand(() -> algaeManipulator.setDeploySpeed(0.125)),
      new MoveToPoseRelative(Units.inchesToMeters(82), 0, 0, 0, drivebase),
      new AutoWait(1500), //
      new MoveElevatorToLevel(elevator.L1, elevator),
      new InstantCommand(()-> feeder.setSpeeds(0.05, 0.25)),
      new AutoWait(1000),
      new InstantCommand(()-> feeder.setSpeeds(0, 0)),
      new AutoWait(500),
      new MoveToPoseRelative(Units.inchesToMeters(-22), 0, 0, 0, drivebase),
      new AutoWait(250),
      //new MoveToPoseRelative(0, 0, 80, 0, drivebase),
      //new RotateAbsolute(0, drivebase),
      //new AutoWait(250),
      new MoveToPoseRelative(0, 0, 80, 0, drivebase),
      //new RotateAbsolute(80, drivebase),
      new AutoWait(250),
      new MoveToPoseRelative(0, 0, 80, 0, drivebase),
      new InstantCommand(()-> drivebase.zeroGyro())
    );
  }
}
