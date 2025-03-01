package frc.robot.Subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog 
    public static class ElevatorIOIn {
        double motor1RawPosition; // Raw position output from encoder
        double motor1OffsetPosition; // Position of motor where 0 represents the lowest position of the elevator
        double motor1Velocity; // Velocity of the motor output
        double motor1DutyCycle; // Duty cycle of motor
        double motor1BusVoltage; // Bus voltage of motor
        double motor1Current;
        boolean motor1Inverted; // Is the motor inverted?


        double motor2RawPosition;
        double motor2OffsetPosition;
        double motor2Velocity;
        double motor2DutyCycle;
        double motor2BusVoltage;
        double motor2Current;
        boolean motor2Inverted;
    }

    public default void updateInputs() {}
}
