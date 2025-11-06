package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MoveElevatorToLevel;
import frc.robot.commands.RotateAbsolute;
import frc.robot.commands.SetStartingHeading;
import frc.robot.subsystems.swervedrive.AlgaeManipulator;
import frc.robot.subsystems.swervedrive.Elevator;
import frc.robot.subsystems.swervedrive.Feeder;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TwoPointReefAuto extends SequentialCommandGroup {

  private int rotationMultiplier;


  public TwoPointReefAuto(double startingYaw, SwerveSubsystem drivebase, Feeder feeder, Elevator elevator, Boolean isLeftSide, Trigger coralDetected, AlgaeManipulator algaeManipulator) {

    // Determine rotation direction depending on side
    // I think this would need to be changed to account for the robot's orientation in this auto idk
    if (isLeftSide) {
      rotationMultiplier = -1;
    } else {
      rotationMultiplier = 1;
    }

    //drivebase.setGyroOffset(startingYaw);
    
    
    addCommands(
      // Step 1: Start by driving diagonally toward nearest trough at 45 degrees
      new SetStartingHeading(startingYaw, drivebase),
      new MoveToPoseRelative(Units.inchesToMeters(-76.5), Units.inchesToMeters(135.5), 0, 0, drivebase),
      new AutoWait(250),
    
       // Step 2: Score coral (raise elevator & run feeder)
      new MoveElevatorToLevel(elevator.L1, elevator),
      new AutoWait(250),
      new InstantCommand(() -> feeder.setSpeeds(0.05, 0.15)),  // Outtake coral
      new AutoWait(1000),
      new InstantCommand(() -> feeder.setSpeeds(0, 0)),
      new MoveElevatorToLevel(elevator.HOPPER, elevator),
      new AutoWait(250),

      // Step 3: Backup from Reef & rotate to face feeder
      new MoveToPoseRelative(Units.inchesToMeters(11), Units.inchesToMeters(-19.5), 0, 0, drivebase),
      //new MoveToPoseRelative(0,0,40 * rotationMultiplier,0,drivebase),
      new RotateAbsolute(10*rotationMultiplier, drivebase),
      //new MoveToPoseRelative(0,0,-40 * rotationMultiplier,0,drivebase),
      //new RotateAbsolute(-80*rotationMultiplier, drivebase),
      //new MoveToPoseRelative(0,0,-80 * rotationMultiplier,0,drivebase),
      //new RotateAbsolute(-80 * rotationMultiplier, drivebase),

      // Step 4: Shift field right (robot-relative Y direction) (idk if its negitive 109 or not)
     // new InstantCommand(() -> drivebase.zeroGyro()),
      new MoveToPoseRelative(0,Units.inchesToMeters(-95)*rotationMultiplier,0,0,drivebase),
      new AutoWait(250),
      
      // Step 5: Drive forward to feeder 
      new MoveToPoseRelative(Units.inchesToMeters(-160),0,0,0,drivebase),
      new AutoWait(250)

      /* 
      // Step 6: Get closer to feeder
      //new MoveToPoseRelative(0,0,40 * rotationMultiplier,0,drivebase),
      new RotateAbsolute(-110*rotationMultiplier, drivebase),
      new AutoWait(250),
      //new InstantCommand(() -> drivebase.zeroGyro()),
      new AutoWait(250),
      new MoveToPoseRelative(Units.inchesToMeters(5),Units.inchesToMeters(8.86),0,0,drivebase),
      

      // Step 7: Intake coral (Don't know the timing of the feeder at all)
      new InstantCommand(() -> feeder.setSpeeds(0.07, 0.07)),  
      new AutoWait(3000, () -> coralDetected.getAsBoolean()),
      new InstantCommand(() -> feeder.setSpeeds(0, 0)),
      
      // Step 8: Drive back to reef
      new MoveToPoseRelative(Units.inchesToMeters(76.5),Units.inchesToMeters(-135.5),0,0,drivebase),
      new AutoWait(250),

      // Step 9: Score coral again
      new MoveElevatorToLevel(elevator.L1, elevator),
      new AutoWait(250),
      new InstantCommand(() -> feeder.setSpeeds(0.05, 0.15)),   // Outtake coral
      new AutoWait(1000),
      new InstantCommand(() -> feeder.setSpeeds(0, 0)),
      new MoveElevatorToLevel(elevator.HOPPER, elevator),
      // Re-zero gyro at the end for clean state (if needed)
      //new InstantCommand(() -> drivebase.zeroGyro()) 

      //Step 10: Move Back to Coral Station For Drivers
      new MoveToPoseRelative(Units.inchesToMeters(-76.5), Units.inchesToMeters(-135.5),0,0, drivebase),

      new InstantCommand(() -> algaeManipulator.setDeploySpeed(0.05))
      */
    );
  }
}