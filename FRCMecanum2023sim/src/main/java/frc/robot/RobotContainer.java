// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

//subsystems
import frc.robot.subsystems.Drivetrain;

//commands
import frc.robot.commands.Drive;
import frc.robot.commands.Auto.DoNothing;
//dashboard
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  //Joystick declare
  private final XboxController  driverJoystick = new XboxController(Constants.Controllers.DRIVER_JOYSTICK);
  private final XboxController operatorJoystick = new XboxController(Constants.Controllers.OPERATOR_JOYSTICK);

  //Subsystems declare
  private final Drivetrain driveTrain = new Drivetrain();


  //Dashboard declare
  //useFOD = SmartDashboard.getBoolean("Use FOD");


  //Commands declare
  private final Drive drive 
    = new Drive(
        driveTrain, 
        () -> driverJoystick.getLeftY(),
        () -> driverJoystick.getLeftX(),
        () -> driverJoystick.getRightX(),
        true);

  private final DoNothing doNothing = new DoNothing();

 
//Sendable chooser declare
SendableChooser<Command> autoChooser = new SendableChooser<>();  //allows for autonomous selection


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings

    //set autonomous selector
    autoChooser.setDefaultOption(
      "Do Nothing", 
      new DoNothing()
      );

    configureButtonBindings();

    // set default commands on subsystems
    driveTrain.setDefaultCommand(drive);

    //Initialize/calibrate gyro
    driveTrain.resetGyro();
    driveTrain.calibrateGyro();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  //Configure dashboard button for FOD


    /*
     * TODO: find values for dpad; create command (method?) to face 0, 90, 180, 270
     */

    //JoystickButton driverLeft = new JoystickButton(driverJoystick, Constants.DRIVER_LEFT);
    //JoystickButton driverRight = new JoystickButton(driverJoystick, Constants.DRIVER_RIGHT);
    //JoystickButton driverUp = new JoystickButton(driverJoystick, Constants.DRIVER_UP);
    //JoystickButton driverDown = new JoystickButton(driverJoystick, Constants.DRIVER_DOWN);
    //JoystickButton driverShoulderTopLeft = new JoystickButton(driverJoystick, Constants.DRIVER_SHOULDER_TOP_LEFT);
    //JoystickButton driverShoulderTopRight = new JoystickButton(driverJoystick, Constants.DRIVER_SHOULDER_TOP_RIGHT);
    //JoystickButton driverShoulderBottomLeft = new JoystickButton(driverJoystick, Constants.DRIVER_SHOULDER_BOTTOM_LEFT);
    //JoystickButton driverShoulderBottomRight = new JoystickButton(driverJoystick, Constants.DRIVER_SHOULDER_BOTTOM_RIGHT);
    //JoystickButton driverLeftJoystick = new JoystickButton(driverJoystick, Constants.DRIVER_LEFT_JOYSTICK);
    //JoystickButton driverRightJoystick = new JoystickButton(driverJoystick, Constants.DRIVER_RIGHT_JOYSTICK);

    //JoystickButton operatorLeft = new JoystickButton(operatorJoystick, Constants.OPERATOR_LEFT);
    //JoystickButton operatorRight = new JoystickButton(operatorJoystick, Constants.OPERATOR_RIGHT);
    //JoystickButton operatorUp = new JoystickButton(operatorJoystick, Constants.OPERATOR_UP);
    //JoystickButton operatorDown = new JoystickButton(operatorJoystick, Constants.OPERATOR_DOWN);
    //JoystickButton operatorShoulderTopLeft = new JoystickButton(operatorJoystick, Constants.OPERATOR_SHOULDER_TOP_LEFT);
    //JoystickButton operatorShoulderTopRight = new JoystickButton(operatorJoystick, Constants.OPERATOR_SHOULDER_TOP_RIGHT);
    //JoystickButton operatorShoulderBottomLeft = new JoystickButton(operatorJoystick, Constants.OPERATOR_SHOULDER_BOTTOM_LEFT);
    //JoystickButton operatorShoulderBottomRight = new JoystickButton(operatorJoystick, Constants.OPERATOR_SHOULDER_BOTTOM_RIGHT);
    //JoystickButton operatorMidLeft = new JoystickButton(operatorJoystick, Constants.OPERATOR_MID_LEFT);
    //JoystickButton operatorLeftJoystick = new JoystickButton(operatorJoystick, Constants.OPERATOR_LEFT_JOYSTICK);
    //JoystickButton operatorRightJoystick = new JoystickButton(operatorJoystick, Constants.OPERATOR_RIGHT_JOYSTICK);
  
      //button command links

      /*  example
      operatorUp.whenPressed(new ShootHigh(shooter)); //set shooter motor to shoot to high goal
      */



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
    return autoChooser.getSelected();
  }


  private void shuffleboardSetup()
  {


  }
  
}
