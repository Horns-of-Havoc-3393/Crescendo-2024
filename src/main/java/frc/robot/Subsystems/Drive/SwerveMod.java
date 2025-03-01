package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.driveConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class SwerveMod {

  ModIOTalon io;
  ModIOInAutoLogged inputs;

  int id;

  LoggedDashboardNumber driveS;
  LoggedDashboardNumber driveV;
  LoggedDashboardNumber driveP;
  LoggedDashboardNumber driveI;
  LoggedDashboardNumber driveD;

  LoggedDashboardNumber steerP;
  LoggedDashboardNumber steerI;
  LoggedDashboardNumber steerD;

  int encoderResets = 0;

  public SwerveMod(TalonFX drive, TalonFX steer, CANcoder absEncoder, Rotation2d absEncoderOffset) {
    id = drive.getDeviceID();

    io = new ModIOTalon(drive, steer, absEncoder, absEncoderOffset);
    inputs = new ModIOInAutoLogged();

    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + drive.getDeviceID(), inputs);

    io.setEncoderOffset((inputs.steerPosRelativePre.minus(inputs.steerPosAbsolute)));
    io.setCurrentLimit(driveConstants.currentLimit);

    driveS = new LoggedDashboardNumber("PIDs/driveS", driveConstants.driveS);
    driveS.set(driveConstants.driveS);
    driveV = new LoggedDashboardNumber("PIDs/driveV", driveConstants.driveV);
    driveV.set(driveConstants.driveV);
    driveP = new LoggedDashboardNumber("PIDs/driveP", driveConstants.driveP);
    driveP.set(driveConstants.driveP);
    driveI = new LoggedDashboardNumber("PIDs/driveI", driveConstants.driveI);
    driveI.set(driveConstants.driveI);
    driveD = new LoggedDashboardNumber("PIDs/driveD", driveConstants.driveD);
    driveD.set(driveConstants.driveD);

    steerP = new LoggedDashboardNumber("PIDs/steerP", driveConstants.driveP);
    steerP.set(driveConstants.steerP);
    steerI = new LoggedDashboardNumber("PIDs/steerI", driveConstants.driveI);
    steerI.set(driveConstants.steerI);
    steerD = new LoggedDashboardNumber("PIDs/steerD", driveConstants.driveD);
    steerD.set(driveConstants.steerD);
  }

  public void periodic() {
    double initial = Logger.getRealTimestamp();
    // io.setDriveVelPID(driveS.get(), driveV.get(), driveP.get(), driveI.get(), driveD.get());
    // io.setSteerPID(steerP.get(), steerI.get(), steerD.get());
    io.updateInputs(inputs);

    if (encoderResets < 10) {
      io.setEncoderOffset((inputs.steerPosRelativePre.minus(inputs.steerPosAbsolute)));
      encoderResets++;
    }
    Logger.processInputs("Drive/Module" + id, inputs);
    Logger.recordOutput("Timers/SwerveModPd", (Logger.getRealTimestamp() - initial) * 0.000001);
  }

  public void updatePIDs() {
    // io.setDriveVelPID(
    //     driveConstants.driveS,
    //     driveConstants.driveV,
    //     driveConstants.driveP,
    //     driveConstants.driveI,
    //     driveConstants.driveD);
    // io.setSteerPID(driveConstants.steerP, driveConstants.driveI, driveConstants.driveD);
    io.setDriveVelPID(driveS.get(), driveV.get(), driveP.get(), driveI.get(), driveD.get());
    io.setSteerPID(steerP.get(), steerI.get(), steerD.get());
  }

  public void setSwerveState(SwerveModuleState state) {
    Logger.recordOutput("Drive/Module" + id + "/targetSpeed", state.speedMetersPerSecond);
    Logger.recordOutput("Drive/Module" + id + "/targetAngle", state.angle);
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, inputs.steerPosRelative);
    Logger.recordOutput(
        "Drive/Module" + id + "/optimizedSpeed", optimizedState.speedMetersPerSecond);
    Logger.recordOutput("Drive/Module" + id + "/optimizedAngle", optimizedState.angle);

    if (Math.abs(optimizedState.speedMetersPerSecond) < 0.1) {
      stop();
    } else {
      io.setDriveSpeed(optimizedState.speedMetersPerSecond);

      double setpoint =
          inputs.steerPosRaw + optimizedState.angle.minus(inputs.steerPosRelative).getRotations();
      io.setSteerPos(setpoint);
    }
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocityMPS, inputs.steerPosRelative);
  }

  public void stop() {
    io.setDriveDutyCycle(0.0);
    io.setSteerDutyCycle(0.0);
  }
}
