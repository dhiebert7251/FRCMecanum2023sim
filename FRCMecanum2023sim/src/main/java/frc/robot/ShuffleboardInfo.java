// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This is a singleton class that will hold all the Shuffleboard entries. This class
 * is where the Shuffleboard layout will be taken care of.  A singleton class is 
 * a class instance where there is only one of them instantiated.  It allows a nice
 * communication method for each of the Subsystems to be able to talk to each other
 * through well-known Network Table entries.
 * 
 * This should enable things like the Drivetrain directing the LED subsystem  
 * bits of information that the LEDs  can communicate to the drive team.
 * 
 * It will take care of giving access to all the various pieces of information
 */
public class ShuffleboardInfo {
    // One tab for each subsystem (or informational piece)
    private final ShuffleboardTab m_driver_tab, m_auto_tab; /*, m_climber_tab, m_intake_tab, m_shooter_tab, m_vision_tab;*/

    // Driver Tab Entries
    private final NetworkTableEntry useFOD; /*, m_IntakeOn, m_Shooting, m_ClimberUp, m_isTargetValid, m_bottomBeam, m_2ndBeam, m_3rdBeam, m_topBeam; */

    // Auto Tab Entries
    private final NetworkTableEntry autoMode;

    // Temporary Tuning values
    //private final NetworkTableEntry m_kpSteer, m_kpDrive;

    
    // private constructors is how you can create a singleton, then provide some 
    // sort of accessor method like getInstance(). Then the getinstance checks
    // whether or not we have an instantiated instance, and if not, then creates 
    // it.

    private static ShuffleboardInfo instance = null;

    private ShuffleboardInfo(){
        // Create the various tabs
        m_auto_tab = Shuffleboard.getTab("Autonomous");
        m_driver_tab = Shuffleboard.getTab("Driver");
        
        //m_climber_tab = Shuffleboard.getTab("Climber");
        //m_intake_tab = Shuffleboard.getTab("Intake");
        //m_shooter_tab = Shuffleboard.getTab("Shooter");
        //m_vision_tab = Shuffleboard.getTab("Vision");

        // Setup the Driver tab
        useFOD = m_driver_tab.add("Use FOD", true)
            .withPosition(0,1)
            .withSize(1, 1)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();

        autoMode = m_auto_tab.add("Autonomous Mode", )
        
        /*
        m_IntakeOn = m_driver_tab.add("Intake On", false)
            .withPosition(1, 1)
            .withSize(1, 1)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();

        */

    }

    public static ShuffleboardInfo getInstance(){
        if( instance == null ){
            instance = new ShuffleboardInfo();
        }

        return instance;
    }

    /*
    public NetworkTableEntry getIntakeOnEntry() {
        return m_IntakeOn;
    }
    */

    /* PID tuning entries
    public NetworkTableEntry getKPsteerEntry() {
        return m_kpSteer;
    }

    public NetworkTableEntry getKPDriveEntry(){
        return m_kpDrive;
    }
    
    */

	public void addAutoChooser(SendableChooser<Command> m_auto_chooser) {
        m_auto_tab.add("Autonomous Chooser", m_auto_chooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1);
	}
}
