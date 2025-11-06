// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RotateAbsolute;
import frc.robot.commands.SetStartingHeading;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateTest extends SequentialCommandGroup {
  /** Creates a new RotateTest. */
  public RotateTest(double startingYaw, SwerveSubsystem driveBase) {
    // Add your commands in the addCommands() call, e.g.
    //driveBase.setGyroOffset(startingYaw);
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetStartingHeading(startingYaw, driveBase),
      new RotateAbsolute(0, driveBase)
      //new AutoWait(1000),
      //new RotateAbsolute(-120, driveBase)
    );
  }
}
