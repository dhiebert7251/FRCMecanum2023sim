// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
  public final static class AutoConstants{
    public static final double AUTO_SPEED = 0.5;
  }

  public final static class PowerConstants{
    public static final int PDP = 0;
  }

  //Controller constants
  public final static class Controllers{
    //controller - Driver
    public static final int DRIVER_JOYSTICK = 0;

    //controller - Driver Axis maps
    public static final int DRIVER_JOYSTICK_LEFT_X_AXIS = 0;
    public static final int DRIVER_JOYSTICK_LEFT_Y_AXIS = 1;
    
    
    /*  
    Remove if using Modified controller - right joystick replaced with potentiometer
    */
    public static final int DRIVER_JOYSTICK_RIGHT_X_AXIS = 2;
    public static final int DRIVER_JOYSTICK_RIGHT_Y_AXIS = 3;
    
    /*
    Use for modified controller only

    public static final int DRIVER_ROTATION = 2; //replaces right x-axis
    */

    //controller - Driver Button maps
    public static final int DRIVER_LEFT = 1;
    public static final int DRIVER_RIGHT = 3;
    public static final int DRIVER_UP = 4;
    public static final int DRIVER_DOWN = 2;
    public static final int DRIVER_SHOULDER_TOP_LEFT = 5;
    public static final int DRIVER_SHOULDER_TOP_RIGHT = 6;
    public static final int DRIVER_SHOULDER_BOTTOM_LEFT = 7;
    public static final int DRIVER_SHOULDER_BOTTOM_RIGHT = 8;
    public static final int DRIVER_LEFT_JOYSTICK = 9;
    public static final int DRIVER_RIGHT_JOYSTICK = 10; 
    //TODO: add D-pad

    //controller - operator
    public static final int OPERATOR_JOYSTICK = 1;

    //controller - Operator Axis maps
    public static final int OPERATOR_JOYSTICK_LEFT_X_AXIS = 0;
    public static final int OPERATOR_JOYSTICK_LEFT_Y_AXIS = 1;
    public static final int OPERATOR_JOYSTICK_RIGHT_X_AXIS = 2;
    public static final int OPERATOR_JOYSTICK_RIGHT_Y_AXIS = 3;

    //controller - Driver Button maps
    public static final int OPERATOR_LEFT = 1;
    public static final int OPERATOR_RIGHT = 3;
    public static final int OPERATOR_UP = 4;
    public static final int OPERATOR_DOWN = 2;
    public static final int OPERATOR_SHOULDER_TOP_LEFT = 5;
    public static final int OPERATOR_SHOULDER_TOP_RIGHT = 6;
    public static final int OPERATOR_SHOULDER_BOTTOM_LEFT = 7;
    public static final int OPERATOR_SHOULDER_BOTTOM_RIGHT = 8;
    public static final int OPERATOR_LEFT_JOYSTICK = 9;
    public static final int OPERATOR_RIGHT_JOYSTICK = 10;
    //TODO: add d-pad
  }

  public final static class Math {
    /** The mathematical constant pi */
    public final static double PI = 3.14159;
    /** Multiply to get from inches to meters */
    public final static double INCHES_2_METERS = 0.0254000;
    /** Multiply to get from meters to inches */
    public final static double METERS_2_INCHES = 39.3701;
  }

    /** Physical chassis measurements and dimensions */
  public final class ChassisConstants {
    
    /*
    TODO: 
    update chassis values to actual robot measurements
    */

    /** Wheel diameter in meters */
    public final static double WHEEL_DIAMETER = 6 * Math.INCHES_2_METERS;
    /** Wheel circumference in meters */
    public final static double WHEEL_CIRCUM = WHEEL_DIAMETER * Math.PI;
    /** Width between two wheels (axle length) in meters */
    public final static double TRACK_WIDTH = 23 * Math.INCHES_2_METERS;
    /** Width of the robot in meters */
    public final static double WIDTH = 27 * Math.INCHES_2_METERS;
    /** Length of the robot in meters */
    public final static double LENGTH = 32.3 * Math.INCHES_2_METERS; 

    /* Wheel locations relative to robot center */

    Translation2d LEFT_FRONT_WHEEL = new Translation2d(0.381, 0.381);
    Translation2d RIGHT_FRONT_WHEEL = new Translation2d(0.381, -0.381);
    Translation2d LEFT_REAR_WHEEL = new Translation2d(-0.381, 0.381);
    Translation2d RIGHT_REAR_WHEEL = new Translation2d(-0.381, -0.381);
    
  }
  
  /** Configuration options, PID constants, and CAN Bus ID's for {@link frc.robot.subsystems.Drivetrain Drivetrain} subsystem */
  public final static class DriveConstants {
    /* 
    TODO:
    Get CANbus ID values for sparkmax controllers
    Check drive motor INVERTED settings
    */

    //Motor IDs
    public static final int LEFT_FRONT_MOTOR = 10;
    public static final int LEFT_REAR_MOTOR = 11;
    public static final int RIGHT_FRONT_MOTOR = 12;
    public static final int RIGHT_REAR_MOTOR = 13;

    //Motor inversion
    public final static boolean LEFT_FRONT_INVERTED = false;
    public final static boolean LEFT_REAR_INVERTED = false;
    public final static boolean RIGHT_FRONT_INVERTED = true;
    public final static boolean RIGHT_REAR_INVERTED = true;

    //SparkMAX PID constants
    /** P constant for SparkMAX onboard PID control */
    public final static double P = 0.0;
    /** I constant for SparkMAX onboard PID control */
    public final static double I = 0.0;
    /** D constant for SparkMAX onboard PID control */
    public final static double D = 0.0;
    /** F constant for SparkMAX onboard PID control */
    public final static double F = 0.0;


    /** Max speed of the robot between [0,1] */
    public final static double MAX_SPEED = 0.75;

    /** Maximum forward driving velocity in m/s (meters per second). Should
     * be slightly lower than robot's maximum free speed */
    public final static double MAX_VELOCITY = 2.0;

    /** Maximum forward driving acceleration in m/s^2 (meters per second
     * squared) */
    public final static double MAX_ACCELERATION = 2.0;

    /** Maximum rotational velocity in rad/s */
    public final static double MAX_ROTATION = Math.PI/4.0;

    /** kS constant with units V (volts) for Drivetrain motor
     * characterization. See
     * <a href=https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html">FRC Docs</a>
     * section on motor characterization for more */
    public final static double S = 1.48;

    /** kV constant with units (Vs)/m (volt seconds per meter) for Drivetrain
     * motor characterization. See
     * <a href=https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html">FRC Docs</a>
       * section on motor characterization for more */
    public final static double V = 2.93;

    /** kA constant with units (Vs^2)/m (volt seconds squared per meter) for
     * Drivetrain motor characterization. See
     * <a href=https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html">FRC Docs</a>
     * section on motor characterization for more */
    public final static double A = 0.0313;

    //encoders
    public final static int kEncoderCPR = 42;
    

    }
    
}
