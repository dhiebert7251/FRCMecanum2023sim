// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

//subsystems
import frc.robot.subsystems.Drivetrain;

//commands
import frc.robot.commands.Drive;
import frc.robot.commands.Auto.Auto2;
import frc.robot.commands.Auto.DoNothing;
import frc.robot.calibration.PIDTuningCommand;

//dashboard
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;



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
  // Shuffleboard Info is the container for all the shuffleboard pieces we want to see
  private ShuffleboardInfo m_sbi_instance;


  //Commands declare
  private final Drive drive
    = new Drive(
        driveTrain, 
        () -> driverJoystick.getLeftY(),
        () -> driverJoystick.getLeftX(),
        () -> driverJoystick.getRightX(),
        true);
  
  //private final DoNothing doNothing = new DoNothing();
  //private final Auto2 auto2 = new Auto2();

  //private final PIDTuningCommand pidTuningCommand = new PIDTuningCommand(driveTrain);

 
//Sendable chooser declare
SendableChooser<Command> autoChooser; //= new SendableChooser<>();  //allows for autonomous selection


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings

    //set autonomous selector
    autoChooser = new SendableChooser<Command>();

    autoChooser.setDefaultOption(
      "Do Nothing", 
      new DoNothing()
      );
    
    //  Add additional auto options
    autoChooser.addOption("Auto 2", new Auto2());

  

    /*  Use shuffleboard for PID tuning
    Shuffleboard.getTab("PID Tuning").add(new ShooterTuningCommand(m_shooter,m_intake)).withWidget(BuiltInWidgets.kCommand);
    */


 

    // set default commands on subsystems
    driveTrain.setDefaultCommand(drive);

    
    //Initialize/calibrate gyro
    driveTrain.resetGyro();
    driveTrain.calibrateGyro();

    //Set up button bindings after subsystems/commands have been declared
    configureButtonBindings();

    //Setup Shuffleboard
    shuffleboardSetup();
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  //Configure dashboard button for FOD

    //JoystickButton driverLeft = new JoystickButton(driverJoystick, Constants.DRIVER_LEFT);
    //JoystickButton driverRight = new JoystickButton(driverJoystick, Constants.DRIVER_RIGHT);
    //JoystickButton driverUp = new JoystickButton(driverJoystick, Constants.DRIVER_UP);
    //JoystickButton driverDown = new JoystickButton(driverJoystick, Constants.DRIVER_DOWN);
    JoystickButton driverShoulderTopLeft = new JoystickButton(driverJoystick, Constants.Controllers.DRIVER_SHOULDER_TOP_LEFT);
    //JoystickButton driverShoulderTopRight = new JoystickButton(driverJoystick, Constants.DRIVER_SHOULDER_TOP_RIGHT);
    //JoystickButton driverShoulderBottomLeft = new JoystickButton(driverJoystick, Constants.DRIVER_SHOULDER_BOTTOM_LEFT);
    //JoystickButton driverShoulderBottomRight = new JoystickButton(driverJoystick, Constants.DRIVER_SHOULDER_BOTTOM_RIGHT);
    //JoystickButton driverLeftJoystick = new JoystickButton(driverJoystick, Constants.DRIVER_LEFT_JOYSTICK);
    //JoystickButton driverRightJoystick = new JoystickButton(driverJoystick, Constants.DRIVER_RIGHT_JOYSTICK);
    //POVButton driverPOVTop = new POVButton(driverJoystick, 0);
    //POVButton driverPOVRight = new POVButton(driverJoystick, 90);
    //POVButton driverPOVBottom = new POVButton(driverJoystick, 180);
    //POVButton driverPOVLeft = new POVButton(driverJoystick, 270);


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
      driverShoulderTopLeft.whenPressed(new DoNothing());
      //driverShoulderTopLeft.whenPressed(doNothing);

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
      // Setup methods for each subset of Shuffleboard needed setup
      m_sbi_instance = ShuffleboardInfo.getInstance();
      
      // Setup Autonomous
      m_sbi_instance.addAutoChooser(autoChooser);
      
      // setupClimberShuffleBoard(); call other shuffleboard setups
      setupAutonomousShuffleboard();
      setupPidTuningCommandShuffleboard();

    }

    @SuppressWarnings("unused")
    private void setupAutonomousShuffleboard(){
      SmartDashboard.putData("Autonomous", new DoNothing());
      SmartDashboard.putData("Autonomous", new Auto2());

      /* should this be..
      SmartDashboard.putData("Do Nothing", new DoNothing());
      SmartDashboard.putData("Auto 2", new Auto2());

      */

      /* or..
      SmartDashboard.getTab("Autonomous").add("Do Nothing", newDoNothing());
      SmartDashboard.getTab("Autonomous").add("Auto 2", new Auto2());

      */
      
    }

    @SuppressWarnings("unused")
    private void setupPidTuningCommandShuffleboard(){
      // First, assign a local variable the Tab that we are going to use
      // for pid tuning in Shuffleboard
      Shuffleboard.getTab("PID Tuning").add(new PIDTuningCommand(driveTrain));

    }

  /*  Sample for mechanism

  private void setupClimberShuffleBoard(){
    Shuffleboard.getTab("Climber").add("up",new ClimberUp(m_climber));
    Shuffleboard.getTab("Climber").add("down",new ClimberDown(m_climber));
    Shuffleboard.getTab("Climber").add("Lift up and fire Solenoid",new ClimbAndLock(m_climber));
    Shuffleboard.getTab("Climber").add("ExtendSolenoid",new ExtendClimberSolenoid(m_climber));
    Shuffleboard.getTab("Climber").add("RetractSolnoid",new RetractClimberSolenoid(m_climber));
  }
  
  */

  @SuppressWarnings("unused")
  private void setupDriveMMShuffleboard(){
    // First, assign a local variable the Tab that we are going to use
    // for pid tuning in Shuffleboard
    
    //Shuffleboard.getTab("Drive MM").add(new DriveMM(m_drive_train, 100)).withPosition(0, 0);
    //Shuffleboard.getTab("Drive MM").add(new DriveMM(m_drive_train, -100)).withPosition(2, 0);
    //SmartDashboard.putData("Drive 100", new DriveMM(m_drive_train, 100));
    //SmartDashboard.putData("Drive -100", new DriveMM(m_drive_train, -100));
  }


  
}
