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

    public static final double maxSpeedMPS = 0.6;
    public static final double maxRotRPS = 5;
    public static final double lateralAccelLimitMPSPS = 4;
    public static final double rotationalAccelLimitRPSPS = 20;
    public static final double deadZone = 0.01;

    public static final double steeringRatio =
        21.42857142857143; // Rotations of motor per rotations of wheel
    public static final double wheelRadius = 1.932; // inches
    public static final double driveRatio = 5.903;

    public static final Rotation2d[] absoluteEncoderOffsets = {
      Rotation2d.fromRadians(0.332),
      Rotation2d.fromRadians(-0.664),
      Rotation2d.fromRadians(1.921),
      Rotation2d.fromRadians(-0.823)
    };

    public static final Translation2d[] offsets = {
      new Translation2d(0.238125, -0.238125),
      new Translation2d(-0.238125, -0.238125),
      new Translation2d(-0.238125, 0.238125),
      new Translation2d(0.238125, 0.238125)
    };

    public static double driveS = 0;
    public static double driveV = 0.01;
    public static double driveP = 0.03;
    public static double driveI = 0.0;
    public static double driveD = 0.0;

    public static double steerP = 29;
    public static double steerI = 0.001;
    public static double steerD = 0.2;
  }

  public static final class shooterConstants {

    public static int shootBeamID = 0;
    public static int intakeBeamID = 1;
    public static double shooterSpeed = 15;

    public static double shootWheelDiameter = 4.0;

    public static double elevatorConversion = 74 / 14 * 50;

    public static double shooterP = 0;
    public static double shooterI = 0;
    public static double shooterD = 0;

    public static double elevatorP = 0;
    public static double elevatorI = 0;
    public static double elevatorD = 0;
    public static double elevatorG = 0;

    public static double conveyorP = 0;
    public static double conveyorI = 0;
    public static double conveyorD = 0;
    public static double conveyorFF = 0;

    public static Rotation2d elevatorOffset =
        Rotation2d.fromRadians(2.924).minus(Rotation2d.fromDegrees(180));
    public static double shooterFF = 0.0095;
  }

  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
