package frc.robot.Subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {
    @AutoLog
    public static class ManipulatorIOIn {
        boolean beamBreakFwd; // True when the beam is broken at the front of the manipulator
        boolean beamBreakBack; // True when the beam is broker at the back of the manipulator

        double wristCalculatedPosition; // Calculated position of the wrist mechanism
        double wristMotorOffsetPosition; // Position of motor shaft where 0 is the same position as wristCalculatedPosition's 0
        double wristMotorRawPosition; // Raw output from getPosition function
        double wristMotorAngularVelocity; // Velocity of motor output
        double wristMotorDutyCycle;
        double wristMotorCurrent;
        double wristMotorBusVoltage; // Voltage on the bus

        double rollerAngularVelocity; // Angular velocity of rollers in RPM
        double rollerMotorAngularVelocity; // Angular velocity of motor in RPM
        double rollerSurfaceVelocity; // Surface velocity of rollers in M/s
        double rollerMotorDutyCycle;
        double rollerMotorCurrent;
        double rollerMotorBusVoltage; // Voltage on the bus
    }

    public default void updateInputs() {}
}
