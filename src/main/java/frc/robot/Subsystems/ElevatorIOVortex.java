package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;


public class ElevatorIOVortex implements ElevatorIO {
    SparkMax motor1;
    SparkMax motor2;

    RelativeEncoder encoder1;
    RelativeEncoder encoder2;

    SparkMaxConfig config = new SparkMaxConfig();
    ClosedLoopConfig pidConfig;

    double encoderOffset1;
    double encoderOffset2;

    SparkMaxConfigAccessor motor1ConfAccessor;
    SparkMaxConfigAccessor motor2ConfAccessor;
    
    public ElevatorIOVortex(SparkMax motor1, SparkMax motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;

        encoder1 = motor1.getEncoder();
        encoder2 = motor2.getEncoder();



        // Configure Motors ---------------------------------------------------------------
        pidConfig = new ClosedLoopConfig();
        pidConfig.pid(0,0,0);
        config.apply(pidConfig);
        config.idleMode(IdleMode.kBrake);

        applyConfig();
        // --------------------------------------------------------------------------------


    }

    public void updateInputs(ElevatorIOIn inputs) {
        inputs.motor1RawPosition = encoder1.getPosition();
        inputs.motor1OffsetPosition = inputs.motor1RawPosition - encoderOffset1;

        inputs.motor1Velocity = encoder1.getVelocity();

        inputs.motor1DutyCycle = motor1.getAppliedOutput();
        inputs.motor1BusVoltage = motor1.getBusVoltage();
        inputs.motor1Current = motor1.getOutputCurrent();
        inputs.motor1Inverted = false;



        inputs.motor2RawPosition = encoder2.getPosition();
        inputs.motor2OffsetPosition = inputs.motor2RawPosition - encoderOffset2;

        inputs.motor2Velocity = encoder2.getVelocity();

        inputs.motor2DutyCycle = motor2.getAppliedOutput();
        inputs.motor2BusVoltage = motor2.getBusVoltage();
        inputs.motor2Current = motor2.getOutputCurrent();
        inputs.motor2Inverted = true;
    }

    private void applyConfig() {
        motor1.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motor2.configure(config.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // sets the position of both motors *in motor rotations* (0 is the bottom position)
    public void setPosition(double position) {
        motor1.getClosedLoopController().setReference(position+encoderOffset1, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.2, ArbFFUnits.kVoltage);
        motor2.getClosedLoopController().setReference(position+encoderOffset2, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.2, ArbFFUnits.kVoltage);
    }

    public void stopElevator() {
        motor1.stopMotor();
        motor2.stopMotor();
    }

    public void setSpeed(double speed1, double speed2) {
        motor1.set(speed1);
        motor2.set(speed2);
    }

    public void setVoltage(double voltage1, double voltage2) {
        motor1.setVoltage(voltage1);
        motor2.setVoltage(voltage2);
    }

    public void updatePIDs(double P, double I, double D, double FF, double Vel, double Accel, double Jerk) {
        pidConfig.pidf(P, I, D, FF);
        pidConfig.maxMotion.maxVelocity(Vel);
        pidConfig.maxMotion.maxAcceleration(Accel);
        config.apply(pidConfig);
        applyConfig();
    }

    public void setBrake(boolean enabled) {
        IdleMode mode = (enabled ? IdleMode.kBrake : IdleMode.kCoast);
        config.idleMode(mode);
        applyConfig();
    }


    // Shifts offsets so that the elevator's current position is reported as whatever is input to the "position" argument
    // (calling "setCurrentPosition(0);" will zero the elevator at the current position)
    public void setCurrentPosition(double position) {
        encoderOffset1 = encoder1.getPosition()-position;
        encoderOffset2 = encoder2.getPosition()-position;
    }
}
