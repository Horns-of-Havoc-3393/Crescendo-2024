package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOIn {
    double shooter1Volts;
    double shooter1DutyCycle;
    double shooter1SpeedRPS;
    double shooter1SpeedMPS;
    double shooter1SetpointRPS;
    double shooter1SetpointMPS;

    double shooter2Volts;
    double shooter2DutyCycle;
    double shooter2SpeedRPS;
    double shooter2SpeedMPS;
    double shooter2SetpointRPS;
    double shooter2SetpointMPS;

    double elevator1Volts;
    double elevator1DutyCycle;
    double elevator1SpeedRPS;
    double elevator1SetpointDeg;
    double elevator1Position;
    double elevator1PositionPre;

    double elevator2Volts;
    double elevator2DutyCycle;
    double elevator2SpeedRPS;
    double elevator2SetpointDeg;
    double elevator2Position;
    double elevator2PositionPre;

    double conveyorVolts;
    double conveyorDutyCycle;
    double conveyorSpeedRPS;

    double intakeVolts;
    double intakeDutyCycle;
    double intakeSpeedRPS;

    Rotation2d shooterAngleAbs;

    boolean beamBreak;

    double noteSpeed;
  }

  public default void updateInputs(ShooterIOIn inputs) {}
}
