package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModIO {
  @AutoLog
  public static class ModIOIn {
    double driveVelocityRPS;
    double driveVelocityMPS;
    double steerVelocityRPS;
    double driveCurrentAmps;
    double steerCurrentAmps;
    double driveVolts;
    double steerVolts;
    Rotation2d steerPosRelative;
    Rotation2d steerPosRelativePre;
    Rotation2d steerPosAbsolute;
    double steerPosRaw;
    double driveVelErr;
    double steerPosErr;
    double driveDutyCycle;
    double currentLimit;
    Rotation2d targetAngle;
    double targetSpeed;
  }

  public default void updateInputs(ModIOIn inputs) {}
}
