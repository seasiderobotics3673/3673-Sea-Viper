// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import static frc.robot.Constants.MAX_ANGULAR_SPEED;
import static frc.robot.Constants.MAX_SPEED;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private double counter;

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //private Vision m_vision;

  private Timer disabledTimer;

  //private PhotonCamera camera = new PhotonCamera("centerCam");
  private PhotonCamera visionCamera;

  private Cameras cameraEnum = Cameras.CENTER_CAM;

  private String cameraName = cameraEnum.name();

  



  UsbCamera cam0;

  //https://rgbcolorpicker.com/0-1
  //private final Color GreenTarget = new Color(0.197, 0.561, 0.240);
  //private final Color RedTarget = new Color(0.561, 0.232, 0.114);
  //private final Color YellowTarget = new Color(0.361, 0.524, 0.113);
  //private final Color BlueTarget = new Color(0.143, 0.427, 0.429);

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    cam0 = CameraServer.startAutomaticCapture(); //Main Viewing USB Camera
    //System.out.println("cam0 isConnected = " + cam0.isConnected());
    //System.out.println("cam0 isEnabled = " + cam0.isEnabled());
    cam0.setFPS(15);
    cam0.setResolution(320, 240);

    //colorMatcher.addColorMatch(whiteTarget);
    //colorMatcher.addColorMatch(background);

    visionCamera = new PhotonCamera(cameraName);

    

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    counter++;

    //Color detectedColor = colorSensor.getColor();
    //String colorString;
    //ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    /*
    if (match.color == whiteTarget) {
      colorString = "White";
    } else if (match.color == background) {
      colorString = "Background";
    } else {
      colorString = "Unknown";
    }
      */
      
    //SmartDashboard.putNumber("Confidence", match.confidence);
    //SmartDashboard.putString("Detected Color", colorString);

    if (counter > 20) {
      counter = 0;
    //  System.out.println("Proximity: " + proximity);
    }
      /*
      System.out.print("Red: " + detectedColor.red);
      System.out.print(" Green: " + detectedColor.green);
      System.out.print(" Blue: " + detectedColor.blue);
      System.out.print(" Color: " + colorString);
      System.out.println(" Confidence: " + match.confidence);

      */
      


    }

    
  

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      //disabledTimer.reset();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    //m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.println("m_autonomousCommand.getname() = " + m_autonomousCommand.getName());

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {

    if (visionCamera != null) {
      double forward = -m_robotContainer.logitechController.getLeftY() * Constants.MAX_SPEED;
      double strafe = -m_robotContainer.logitechController.getLeftX() * Constants.MAX_SPEED;
      double turn = -m_robotContainer.logitechController.getRightX() * Constants.MAX_ANGULAR_SPEED;
      

      boolean targetVisible = false;
      double targetYaw = 0.0;
      //var results = camera.getAllUnreadResults();
      //var results = Cameras.CENTER_CAM.camera.getAllUnreadResults();
      var results = cameraEnum.camera.getAllUnreadResults();
      if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == 14) {
                    // Found Tag 14, record its information
                    targetYaw = target.getYaw();
                    targetVisible = true;
                }
            }
        }
          
    }

    if (m_robotContainer.logitechController.a().getAsBoolean() && targetVisible) {
      // Driver wants auto-alignment to tag 14
              // And, tag 14 is in sight, so we can turn toward it.
              // Override the driver's turn command with an automatic one that turns toward the tag.
              turn = -1.0 * targetYaw * Constants.VISION_TURN_kP * Constants.MAX_ANGULAR_SPEED;
              System.out.println(turn);

    }


            // Command drivetrain motors based on target speeds
            m_robotContainer.drivebase.drive(new Translation2d(forward, strafe), turn, true);

            // Put debug information to the dashboard
            SmartDashboard.putBoolean("Is Target Visible?: ", targetVisible);



    }
    
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }

}
/*
  //2-5-25 Limelight Vision Code Stuff;
  //https://github.com/LimelightVision/limelight-examples/tree/main/java-wpilib/swerve-aim-and-range

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    //targetingAngularVelocity *= Drivetrain.kMaxAngularSpeed;
    targetingAngularVelocity *= MAX_ANGULAR_SPEED;

    //invert since tx is positive when                                          the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    //targetingForwardSpeed *= Drivetrain.kMaxSpeed;
    targetingForwardSpeed *= MAX_SPEED;
    
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

}
*/