package frc.robot.Positioning;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface PosIO {
  @AutoLog
  public static class PosIOIn {
    public Rotation2d xGyro = new Rotation2d(0.0);
    public Rotation2d yGyro = new Rotation2d(0.0);
    public Rotation2d zGyro = new Rotation2d(0.0);

    public double xAccel = 0.0;
    public double yAccel = 0.0;
    public double zAccel = 0.0;

    public double kGain = 0.0;
    public double estX = 0.0;
    public double estY = 0.0;
    public double estZ = 0.0;
  }

  public default void updateInputs(PosIOIn inputs) {}
}
