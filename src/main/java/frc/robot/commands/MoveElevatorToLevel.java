// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorToLevel extends Command {
  /** Creates a new MoveElevatorToLevel. */
  private int level;
  private Elevator elevator;
  private boolean isFinishedFlag;
  private int counter;
  public MoveElevatorToLevel(int level, Elevator elevator) {
    this.level = level;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.goToPreset(level);
    isFinishedFlag = false;
  }



  // Called every time the scheduler run%s while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    if (counter >= 10) {
      counter = 0;
    }
    /*
    if(elevator.getHeightInTicks() < elevator.getDestinationInTicks() - Constants.ELEVATOR_TOLERANCE) {
      elevator.setMotorSpeed(Constants.ELEVATOR_SPEED_UP);
      if (counter == 0) {
        System.out.println("getHeightInTicks = " + elevator.getHeightInTicks());
        System.out.print("getDestinationInTicks = " + elevator.getDestinationInTicks());
      }
    } else {
      if(elevator.getHeightInTicks() > elevator.getDestinationInTicks() + Constants.ELEVATOR_TOLERANCE) {
        elevator.setMotorSpeed(-Constants.ELEVATOR_SPEED_DOWN);
        if (counter == 0) {
          System.out.println("getHeightInTicks = " + elevator.getHeightInTicks());
          System.out.print("getDestinationInTicks = " + elevator.getDestinationInTicks());
        }
      } else {
          elevator.setMotorSpeed(Constants.ELEVATOR_ANTIGRAVITY);
          System.out.println("MoveElevatorToLevel Is Finished Flag = True");
          isFinishedFlag = true;
      }
    }
      */
    
    if (elevator.getHeightInTicks() < elevator.getDestinationInTicks() - Constants.ELEVATOR_RAMPDOWN - Constants.ELEVATOR_TOLERANCE) {
      elevator.setMotorSpeed(Constants.ELEVATOR_SPEED_UP);
    } else {
      if (elevator.getHeightInTicks() < elevator.getDestinationInTicks() - Constants.ELEVATOR_TOLERANCE) {
        elevator.setMotorSpeed(Constants.ELEVATOR_SPEED_UP_SLOW);
        if (counter == 0) {
          System.out.println("getHeightInTicks = " + elevator.getHeightInTicks());
          //System.out.println("getDestinationInTicks = " + elevator.getDestinationInTicks());
        }
      } else {
        if (elevator.getHeightInTicks() > elevator.getDestinationInTicks() + Constants.ELEVATOR_RAMPDOWN + Constants.ELEVATOR_TOLERANCE) {
          elevator.setMotorSpeed(-Constants.ELEVATOR_SPEED_DOWN);
        } else {
          if (elevator.getHeightInTicks() > elevator.getDestinationInTicks() + Constants.ELEVATOR_TOLERANCE) {
            elevator.setMotorSpeed(-Constants.ELEVATOR_SPEED_DOWN_SLOW);
            if (counter == 0) {
              System.out.println("getHeightInTicks = " + elevator.getHeightInTicks());
              //System.out.println("getDestinationInTicks = " + elevator.getDestinationInTicks());
            }
          } else {
            elevator.setMotorSpeed(Constants.ELEVATOR_ANTIGRAVITY);
            System.out.println("MoveElevatorToLevel Is Finished Flag = True");
            isFinishedFlag = true;
          }
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinishedFlag;
  }
}
