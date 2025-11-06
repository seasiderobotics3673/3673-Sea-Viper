// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.AlgaeManipulator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMoveForward extends SequentialCommandGroup {
  /** Creates a new AutoMoveForward. */
  public AutoMoveForward(SwerveSubsystem drivebase, AlgaeManipulator algaeManipulator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> algaeManipulator.setDeploySpeed(0.125)),
      new AutoWait(250),
      new MoveToPoseRelative(Units.inchesToMeters(36), 0, 0, 0, drivebase)




    );
  }
}
