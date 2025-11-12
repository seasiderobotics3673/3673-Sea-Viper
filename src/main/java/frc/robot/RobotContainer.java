// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.platform.can.AutocacheState;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveElevatorToLevel;
import frc.robot.commands.MoveToVisionTarget;
import frc.robot.commands.SlowDown;
import frc.robot.commands.swervedrive.auto.AutoDoNothing;
import frc.robot.commands.swervedrive.auto.AutoMoveForward;
import frc.robot.commands.swervedrive.auto.MoveToPoseRelative;
import frc.robot.commands.swervedrive.auto.RotateTest;
import frc.robot.commands.swervedrive.auto.ScoreCoralInReef;
import frc.robot.commands.swervedrive.auto.SideWallsScoreCoralInReef;
import frc.robot.commands.swervedrive.auto.TwoPointReefAuto;
import frc.robot.subsystems.swervedrive.AlgaeManipulator;
import frc.robot.subsystems.swervedrive.Elevator;
import frc.robot.subsystems.swervedrive.Feeder;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

import java.io.File;
import java.time.Instant;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

//import swervelib.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  //final         CommandJoystick elevatorOperator = new CommandJoystick(1);
  final         CommandJoystick brodieBox2025 = new CommandJoystick(4);

  final         CommandXboxController logitechController = new CommandXboxController(2);
  // The robot's subsystems and commands are defined here...
  protected final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  private final Elevator elevator = new Elevator();
  
  private final Feeder feeder = new Feeder();

  private final AlgaeManipulator algaeManipulator = new AlgaeManipulator();

  protected final Vision vision = new Vision(() -> drivebase.getSwerveDrive().getPose(), drivebase.getSwerveDrive().field);

  private final Trigger elevatorHighLimit = new Trigger(()-> elevator.isAtHighLimit());

  private final Trigger elevatorLowLimit = new Trigger(()-> elevator.isAtLowLimit());

  private final Trigger coralDetector = new Trigger(()-> feeder.hasCoralDisappeared());

  private Cameras cameraEnum = Cameras.CENTER_CAM;


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

   private final SendableChooser<Command> autoChooser = new SendableChooser<>();


  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    //registerNamedCommands(); - Important, no idea what this does looking at old code
    configureBindings();

    autoChooser.setDefaultOption("AutoDoNothing", new AutoDoNothing(180, drivebase, algaeManipulator));
    autoChooser.addOption("Score Coral In Reef", new ScoreCoralInReef(180, drivebase, feeder, elevator, algaeManipulator));
    autoChooser.addOption("Score Coral In Reef (Left Wall)", new SideWallsScoreCoralInReef(-120, drivebase, feeder, elevator, algaeManipulator, true));
    autoChooser.addOption("Score Coral In Reef (Right Wall)", new SideWallsScoreCoralInReef(120, drivebase, feeder, elevator, algaeManipulator, false));
    autoChooser.addOption("Move Forward", new AutoMoveForward(drivebase, algaeManipulator));
    //autoChooser.addOption("Move Forward", new TestMovement1(drivebase));
    //autoChooser.addOption("Two Point Reef Auto (Left)", new TwoPointReefAuto(-120, drivebase, feeder, elevator, true, coralDetector, algaeManipulator));
    //autoChooser.addOption("Two Point Reef Auto (Right)", new TwoPointReefAuto(120, drivebase, feeder, elevator, false, coralDetector, algaeManipulator));
    //autoChooser.addOption("Rotate Test", new RotateTest(180, drivebase));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    //drivebase.setDefaultCommand(new SwerveDrive(null, null, 0, null));



  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    //Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocity = new SequentialCommandGroup(new InstantCommand(() -> drivebase.setSpeedMultipler(1.0)), drivebase.driveFieldOriented(driveAngularVelocity));
    //Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveRobotOrientedAngularVelocity = new SequentialCommandGroup(new InstantCommand(() -> drivebase.setSpeedMultipler(-1.0)), drivebase.driveFieldOriented(driveRobotOriented));
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      //drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
      
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().onTrue((Commands.runOnce(drivebase::zeroGyro)));

      //UMARK LATER; driverXbox.a().onTrue(driveRobotOrientedAngularVelocity).onFalse(driveFieldOrientedAnglularVelocity);

      /*
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      */
      driverXbox.start().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.back().onTrue(new InstantCommand(() -> elevator.printEncoderValue()));
      driverXbox.leftBumper().onTrue(new MoveToVisionTarget(vision, drivebase));
      //driverXbox.rightBumper().onTrue(Commands.none());
      //Set Speed to Slow Mode or Back To Normal
      //driverXbox.rightBumper().onTrue(new InstantCommand(() -> drivebase.toggleSpeedMultiplier()));
      driverXbox.rightBumper().toggleOnTrue(new SlowDown(drivebase));
        //.toggleOnTrue(new InstantCommand(() -> drivebase.setSpeedMultipler(Constants.SWERVE_SPEED_FULL)))
        //.toggleOnFalse(new InstantCommand(() -> drivebase.setSpeedMultipler(Constants.SWERVE_SPEED_SLOW)));

      //MoveToPoseRelative Auto Testing
      //elevatorOperator.button(12).onTrue(new SequentialCommandGroup(
        //new MoveToPoseRelative(1.0, 0, 0, 5000, drivebase),
        //new MoveToPoseRelative(0, -1.0, 0, 5000, drivebase)
        //new MoveToPoseRelative(0, 0, 90.0, 5000, drivebase)

      //));
      //L1
      brodieBox2025.button(10).onTrue(new MoveElevatorToLevel(elevator.L1, elevator));
      //L2
      brodieBox2025.button(9).onTrue(new MoveElevatorToLevel(elevator.L2, elevator));
      //L3
      brodieBox2025.button(12).onTrue(new MoveElevatorToLevel(elevator.L3, elevator));
      //L4
      //brodieBox2025.button(11).onTrue(new MoveElevatorToLevel(elevator.L4, elevator));
      //Hopper/Reset
      brodieBox2025.button(6).onTrue(new MoveElevatorToLevel(elevator.HOPPER, elevator));

      //Manual Go Up
      brodieBox2025.button(8)
        .onTrue(new InstantCommand(()-> elevator.changeMotorSpeed(Constants.ELEVATOR_MANUAL_SPEED_UP)))
        .onFalse(new InstantCommand(()-> elevator.changeMotorSpeed(Constants.ELEVATOR_MANUAL_SPEED_UP_INVERSE)));
      
      //Manual Go Down
      brodieBox2025.button(7)
      .onTrue(new InstantCommand(()-> elevator.changeMotorSpeed(Constants.ELEVATOR_MANUAL_SPEED_DOWN)))
      .onFalse(new InstantCommand(()-> elevator.changeMotorSpeed(Constants.ELEVATOR_MANUAL_SPEED_DOWN_INVERSE)));

      //Feeder Reversed/Out
      brodieBox2025.button(1)
        .onTrue(new InstantCommand(()-> feeder.setSpeeds(-0.07, -0.07))) 
        .onFalse(new InstantCommand(()-> feeder.setSpeeds(0.0, 0.0)));

      //Feeder Shoot/In
      brodieBox2025.button(3)
        .onTrue(new InstantCommand(()-> feeder.setSpeeds(0.07, 0.07))) 
        .onFalse(new InstantCommand(()-> feeder.setSpeeds(0.0, 0.0)));
      
      //Feeder Curved Shoot
      brodieBox2025.button(2)
        .onTrue(new InstantCommand(()-> feeder.setSpeeds(0.05, 0.15)))
        .onFalse(new InstantCommand(()-> feeder.setSpeeds(0.0, 0.0)));
      
      //Belt Set Speed
      brodieBox2025.button(4)
        .onTrue(new InstantCommand(()-> algaeManipulator.setBeltSpeed(0.25)))
        .onFalse(new InstantCommand(()-> algaeManipulator.setBeltSpeed(0)));
      //Deploy Speed (Keep Low)
      brodieBox2025.button(5)
        .onTrue(new InstantCommand(()-> algaeManipulator.setDeploySpeed(0.125)))
        .onFalse(new InstantCommand(()-> algaeManipulator.setDeploySpeed(0)));

      coralDetector.onTrue(new InstantCommand(()-> feeder.setSpeeds(0.0, 0.0)));

      //Climber Up
      //brodieBox2025.button(5)

      //Climber Down
      //brodieBox2025.button(4)

      //Feeder In
      //brodieBox2025.button(1)

      //Feeder Out
      //brodieBox2025.button(3)

      //Rotate towards apriltag toggle
      logitechController.a()
        .whileTrue(drivebase.aimAtTarget(cameraEnum));


      
      elevatorHighLimit.onTrue(new ParallelCommandGroup(
        //new InstantCommand(()-> elevator.setMotorSpeed(0.0)),
        new InstantCommand(()-> elevator.setMaxEncoderValue())
      ));
      
      elevatorLowLimit.onTrue(new ParallelCommandGroup(
        //new InstantCommand(()-> elevator.setMotorSpeed(0.0)),
        new InstantCommand(()-> elevator.setMinEncoderValue())
      ));
      

      

      //.whileTrue(new InstantCommand(()-> intake.setWinchSpeed(-INTAKE_WINCH_SPEED)))


      
      

    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Auto");
    //drivetrain.resetAllEncoders(); - In Old Code
    //drivetrain.setHeading(0); - In Old Code
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

}
