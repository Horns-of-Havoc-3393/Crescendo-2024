package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants.driveConstants;

public class ModIOTalon implements ModIO {

  TalonFX drive;
  TalonFX steer;

  CANcoder absEncoder;

  StatusSignal<AngularVelocity> driveVelocity;
  StatusSignal<AngularVelocity> steerVelocity;
  StatusSignal<Angle> drivePosition;
  StatusSignal<Current> driveCurrent;
  StatusSignal<Current> steerCurrent;
  StatusSignal<Voltage> driveVolts;
  StatusSignal<Voltage> steerVolts;
  StatusSignal<Angle> steerPosRelative;
  StatusSignal<Angle> steerPosAbsolute;
  StatusSignal<Double> driveVelErr;
  StatusSignal<Double> steerPosErr;
  StatusSignal<Double> driveDutyCycle;

  VelocityDutyCycle driveRequest;
  PositionDutyCycle steerRequest;

  Rotation2d absoluteEncoderOffset = new Rotation2d(0);
  Rotation2d encoderOffset = new Rotation2d(0);

  Rotation2d targetRotation = new Rotation2d(0);
  double targetSpeed = 0.0;

  Slot0Configs dSlot0;
  Slot0Configs sSlot0;

  public ModIOTalon(
      TalonFX drive, TalonFX steer, CANcoder absEncoder, Rotation2d absEncoderOffset) {

    this.absoluteEncoderOffset = absEncoderOffset;

    this.drive = drive;
    this.steer = steer;
    this.absEncoder = absEncoder;

    MotorOutputConfigs outCfg = new MotorOutputConfigs();
    outCfg.NeutralMode = NeutralModeValue.Brake;
    drive.getConfigurator().apply(outCfg);

    outCfg.Inverted = InvertedValue.Clockwise_Positive;
    steer.getConfigurator().apply(outCfg);


    driveVelocity = drive.getVelocity();
    steerVelocity = steer.getVelocity();
    drivePosition = drive.getPosition();
    driveCurrent = drive.getStatorCurrent();
    steerCurrent = steer.getStatorCurrent();
    driveVolts = drive.getMotorVoltage();
    steerVolts = steer.getMotorVoltage();

    FeedbackConfigs configs = new FeedbackConfigs();
    configs.SensorToMechanismRatio = driveConstants.steeringRatio;
    steer.getConfigurator().apply(configs);
    steerPosRelative = steer.getPosition();

    steerPosAbsolute = absEncoder.getAbsolutePosition();

    driveVelErr = drive.getClosedLoopError();
    steerPosErr = steer.getClosedLoopError();

    driveDutyCycle = drive.getDutyCycle();

    dSlot0 = new Slot0Configs();
    sSlot0 = new Slot0Configs();

    driveRequest = new VelocityDutyCycle(0.0);
    steerRequest = new PositionDutyCycle(0.0);
  }

  public void updateInputs(ModIOIn inputs) {
    BaseStatusSignal.refreshAll(
        driveVelocity,
        steerVelocity,
        driveCurrent,
        steerCurrent,
        driveVolts,
        steerVolts,
        steerPosRelative,
        steerPosAbsolute,
        driveVelErr,
        steerPosErr,
        driveDutyCycle);

    inputs.driveVelocityRPS = driveVelocity.getValueAsDouble();
    inputs.driveVelocityMPS =
        driveVelocity.getValueAsDouble()
            / driveConstants.driveRatio
            * (driveConstants.wheelRadius * 0.0254 * 2 * Math.PI);
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
    inputs.drivePosition = drivePosition.getValueAsDouble();
    inputs.driveVolts = driveVolts.getValueAsDouble();

    inputs.steerVelocityRPS = steerVelocity.getValueAsDouble();
    inputs.steerPosRelative =
        Rotation2d.fromRotations(steerPosRelative.getValueAsDouble()).minus(encoderOffset);

    inputs.steerPosRelativePre = Rotation2d.fromRotations(steerPosRelative.getValueAsDouble());
    inputs.steerCurrentAmps = steerCurrent.getValueAsDouble();
    inputs.steerVolts = steerVolts.getValueAsDouble();

    inputs.steerPosAbsolute =
        Rotation2d.fromRotations(steerPosAbsolute.getValueAsDouble()).minus(absoluteEncoderOffset);

    inputs.steerPosRaw = steerPosRelative.getValueAsDouble();

    inputs.driveVelErr = driveVelErr.getValueAsDouble();
    inputs.steerPosErr = steerPosErr.getValueAsDouble();

    inputs.driveDutyCycle = driveDutyCycle.getValueAsDouble();

    inputs.targetAngle = targetRotation;
    inputs.targetSpeed = targetSpeed;
  }

  public void setDriveSpeed(double speedMPS) {
    targetSpeed = speedMPS;
    double outputSpeed =
        speedMPS / (driveConstants.wheelRadius * 0.0254 * 2 * Math.PI) * driveConstants.driveRatio;
    drive.setControl(driveRequest.withVelocity(outputSpeed));
  }

  public void setDriveDutyCycle(double volts) {
    drive.setControl(new DutyCycleOut(volts));
  }

  public void setCurrentLimit(double limit) {
    CurrentLimitsConfigs conf = new CurrentLimitsConfigs();
    conf.SupplyCurrentLimit = limit;
    conf.StatorCurrentLimit = limit;
    conf.StatorCurrentLimitEnable = true;
    conf.SupplyCurrentLimitEnable = true;
    drive.getConfigurator().apply(conf);
  }

  public void setSteerPos(double posRotations) {
    targetRotation = Rotation2d.fromRotations(posRotations);
    steer.setControl(steerRequest.withSlot(0).withPosition(posRotations));
  }

  public void setSteerDutyCycle(double volts) {
    steer.setControl(new DutyCycleOut(volts));
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
