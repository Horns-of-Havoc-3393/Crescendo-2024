package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.driveConstants;

public class ModIOTalon implements ModIO {

  TalonFX drive;
  TalonFX steer;

  CANcoder absEncoder;

  StatusSignal<Double> driveVelocity;
  StatusSignal<Double> steerVelocity;
  StatusSignal<Double> driveCurrent;
  StatusSignal<Double> steerCurrent;
  StatusSignal<Double> driveVolts;
  StatusSignal<Double> steerVolts;
  StatusSignal<Double> steerPosRelative;
  StatusSignal<Double> steerPosAbsolute;

  VelocityDutyCycle driveRequest;
  PositionDutyCycle steerRequest;

  Rotation2d absoluteEncoderOffset = new Rotation2d(0);
  Rotation2d encoderOffset = new Rotation2d(0);

  public ModIOTalon(
      TalonFX drive, TalonFX steer, CANcoder absEncoder, Rotation2d absEncoderOffset) {

    this.absoluteEncoderOffset = absEncoderOffset;

    this.drive = drive;
    this.steer = steer;
    this.absEncoder = absEncoder;

    this.steer.setInverted(true);

    driveVelocity = drive.getVelocity();
    steerVelocity = steer.getVelocity();
    driveCurrent = drive.getStatorCurrent();
    steerCurrent = steer.getStatorCurrent();
    driveVolts = drive.getMotorVoltage();
    steerVolts = steer.getMotorVoltage();

    FeedbackConfigs configs = new FeedbackConfigs();
    configs.SensorToMechanismRatio = driveConstants.steeringRatio;
    steer.getConfigurator().apply(configs);
    steerPosRelative = steer.getPosition();

    steerPosAbsolute = absEncoder.getAbsolutePosition();
  }

  Slot0Configs dSlot0;
  Slot0Configs sSlot0;

  public void updateInputs(ModIOIn inputs) {
    BaseStatusSignal.refreshAll(
        driveVelocity,
        steerVelocity,
        driveCurrent,
        steerCurrent,
        driveVolts,
        steerVolts,
        steerPosRelative,
        steerPosAbsolute);

    inputs.driveVelocityRPS = driveVelocity.getValueAsDouble();
    inputs.driveVelocityMPS =
        driveVelocity.getValueAsDouble()
            / driveConstants.driveRatio
            * (driveConstants.wheelRadius * 0.0254 * 2 * Math.PI);
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
    inputs.driveVolts = driveVolts.getValueAsDouble();

    inputs.steerVelocityRPS = steerVelocity.getValueAsDouble();
    inputs.steerPosRelative =
        Rotation2d.fromRotations(steerPosRelative.getValueAsDouble()).minus(encoderOffset);
    inputs.steerCurrentAmps = steerCurrent.getValueAsDouble();
    inputs.steerVolts = steerVolts.getValueAsDouble();

    inputs.steerPosAbsolute =
        Rotation2d.fromRotations(steerPosAbsolute.getValueAsDouble()).minus(absoluteEncoderOffset);

    inputs.steerPosRaw = steerPosRelative.getValueAsDouble();

    dSlot0 = new Slot0Configs();
    sSlot0 = new Slot0Configs();

    driveRequest = new VelocityDutyCycle(0.0);
    steerRequest = new PositionDutyCycle(0.0);
  }

  public void setDriveSpeed(double speedMPS) {
    double outputSpeed =
        speedMPS / (driveConstants.wheelRadius * 0.0254 * 2 * Math.PI) * driveConstants.driveRatio;
    drive.setControl(driveRequest.withVelocity(outputSpeed));
  }

  public void setDriveVoltage(double volts) {
    drive.setControl(new VoltageOut(volts));
  }

  public void setSteerPos(double posDegrees) {
    steer.setControl(steerRequest.withSlot(0).withPosition(posDegrees));
  }

  public void setSteerVoltage(double volts) {
    steer.setControl(new VoltageOut(volts));
  }

  public void setEncoderOffset(Rotation2d offset) {
    this.encoderOffset = offset;
  }

  public void setDriveVelPID(double s, double v, double p, double i, double d) {
    dSlot0.kS = s;
    dSlot0.kV = v;

    dSlot0.kP = p;
    dSlot0.kI = i;
    dSlot0.kD = d;

    drive.getConfigurator().apply(dSlot0);
  }

  public void setSteerPID(double p, double i, double d) {
    sSlot0.kP = p;
    sSlot0.kI = i;
    sSlot0.kD = d;
    steer.getConfigurator().apply(sSlot0);
  }
}
