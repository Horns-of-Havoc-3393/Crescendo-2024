// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Rotation2d;
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

  public static final class driveConstants {
    public static final double currentLimit = 40; // Decrease when browning out

    // Default values for these constants should be set in initLiveConstants() below
    public static final LoggedNetworkNumber maxSpeedMPS = new LoggedNetworkNumber("/SmartDashboard/Control/LateralMaxSpeed");
    public static final LoggedNetworkNumber maxRotRPS = new LoggedNetworkNumber("/SmartDashboard/Control/AngularMaxSpeed");
    public static final LoggedNetworkNumber lateralAccelLimitMPSPS = new LoggedNetworkNumber("/SmartDashboard/Control/LateralAccel");
    public static final LoggedNetworkNumber rotationalAccelLimitRPSPS = new LoggedNetworkNumber("/SmartDasboard/Control/AngularAccel");


    public static final double deadZone = 0.01;

    public static final double steeringRatio =
        21.42857142857143; // Rotations of motor per rotations of wheel
    public static final double wheelRadius = 1.932; // inches
    public static final double driveRatio = 5.903;

    public static final Rotation2d[] absoluteEncoderOffsets = { // Absolute encoder offsets (do no change unless swerve modules are rebuilt, or wheels are steering all over the place
      Rotation2d.fromRadians(1.890),   // To determine these values, SET THEM ALL TO 0, deploy the code and replace the 0s with "AdvantageKit/Drive/Module{#}/SteerPosAbsolute" for each module
      Rotation2d.fromRadians(-1.213),          
      Rotation2d.fromRadians(-2.163),
      Rotation2d.fromRadians(2.324)
    };

    public static final Translation2d[] offsets = { // translations of each swerve module in the order: {FrontRight, BR, BL, FL}
      new Translation2d(0.238125, -0.238125),      // When editing keep in mind that x is forward and back (+x is forward) and y is side to side (+y is *LEFT*)
      new Translation2d(-0.238125, -0.238125),
      new Translation2d(-0.238125, 0.238125),
      new Translation2d(0.238125, 0.238125)
    };

    public static final Boolean[] driveMotorInversions = {false,false,false,false}; // Set individual motor inversions (FrontRight, BR, BL, FL)

    // Default values for these constants should be set in initLiveConstants() below
    public static final LoggedNetworkNumber driveS = new LoggedNetworkNumber("/SmartDashboard/PIDs/driveS");
    public static final LoggedNetworkNumber driveV = new LoggedNetworkNumber("/SmartDashboard/PIDs/driveV");
    public static final LoggedNetworkNumber driveP = new LoggedNetworkNumber("/SmartDashboard/PIDs/driveP");
    public static final LoggedNetworkNumber driveI = new LoggedNetworkNumber("/SmartDashboard/PIDs/driveI");
    public static final LoggedNetworkNumber driveD = new LoggedNetworkNumber("/SmartDashboard/PIDs/driveD");

    public static final LoggedNetworkNumber steerP = new LoggedNetworkNumber("/SmartDashboard/PIDs/steerP");
    public static final LoggedNetworkNumber steerI = new LoggedNetworkNumber("/SmartDashboard/PIDs/steerI");
    public static final LoggedNetworkNumber steerD = new LoggedNetworkNumber("/SmartDashboard/PIDs/steerD");
  }


  public static final class elevatorConstants {
    public static double wristDriveRatio = 0.1; // Ratio of wrist rotations to motor rotations
    public static double rollerDriveRatio = 1; // Ratio of roller rotations to motor rotations
    public static double rollerDiameter = 2; // Diameter of rollers

    public static final LoggedNetworkNumber elvP = new LoggedNetworkNumber("/SmartDashboard/PIDs/elvP");
    public static final LoggedNetworkNumber elvI = new LoggedNetworkNumber("/SmartDashboard/PIDs/elvI");
    public static final LoggedNetworkNumber elvD = new LoggedNetworkNumber("/SmartDashboard/PIDs/elvD");
    public static final LoggedNetworkNumber elvVel = new LoggedNetworkNumber("/SmartDashboard/PIDs/elvVel");
    public static final LoggedNetworkNumber elvAccel = new LoggedNetworkNumber("/SmartDashboard/PIDs/elvAccel");
    public static final LoggedNetworkNumber elvJerk = new LoggedNetworkNumber("/SmartDashboard/PIDs/elvJerk");

    public static final LoggedNetworkNumber manipP = new LoggedNetworkNumber("/SmartDashboard/PIDs/manipP");
    public static final LoggedNetworkNumber manipI = new LoggedNetworkNumber("/SmartDashboard/PIDs/manipI");
    public static final LoggedNetworkNumber manipD = new LoggedNetworkNumber("/SmartDashboard/PIDs/manipD");
    public static final LoggedNetworkNumber manipFF = new LoggedNetworkNumber("/SmartDashboard/PIDs/manipFF");
  }


  public static void initLiveConstants() {
    driveConstants.maxSpeedMPS.set(0.6);
    driveConstants.maxRotRPS.set(5);
    driveConstants.lateralAccelLimitMPSPS.set(4);
    driveConstants.rotationalAccelLimitRPSPS.set(20);


    driveConstants.driveS.set(0);
    driveConstants.driveV.set(0.01);
    driveConstants.driveP.set(0.03);
    driveConstants.driveI.set(0.0);
    driveConstants.driveD.set(0.0);

    driveConstants.steerP.set(29);
    driveConstants.steerI.set(0.001);
    driveConstants.steerD.set(0.2);

    elevatorConstants.elvP.set(0.1);
    elevatorConstants.elvI.set(0);
    elevatorConstants.elvD.set(0);
    elevatorConstants.elvVel.set(0);
    elevatorConstants.elvAccel.set(0);
    elevatorConstants.elvJerk.set(0);

    elevatorConstants.manipP.set(0.06);
    elevatorConstants.manipI.set(0);
    elevatorConstants.manipD.set(0);
    elevatorConstants.manipFF.set(0);

  }


  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}

