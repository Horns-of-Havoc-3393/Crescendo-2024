package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.driveConstants;
import org.littletonrobotics.junction.Logger;

public class SwerveMod {

  ModIOTalon io;
  ModIOInAutoLogged inputs;

  SwerveModuleState savedState;

  int id;


  int encoderResets = 0;

  public SwerveMod(TalonFX drive, TalonFX steer, CANcoder absEncoder, Rotation2d absEncoderOffset) {
    id = drive.getDeviceID();

    io = new ModIOTalon(drive, steer, absEncoder, absEncoderOffset);
    inputs = new ModIOInAutoLogged();

    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + drive.getDeviceID(), inputs);

    io.setEncoderOffset((inputs.steerPosRelativePre.minus(inputs.steerPosAbsolute)));
    io.setCurrentLimit(driveConstants.currentLimit);

  }

  public void periodic() {
    double initial = RobotController.getFPGATime();
    // io.setDriveVelPID(driveS.get(), driveV.get(), driveP.get(), driveI.get(), driveD.get());
    // io.setSteerPID(steerP.get(), steerI.get(), steerD.get());
    io.updateInputs(inputs);

    if (encoderResets < 10) {
      io.setEncoderOffset((inputs.steerPosRelativePre.minus(inputs.steerPosAbsolute)));
      encoderResets++;
    }
    Logger.processInputs("Drive/Module" + id, inputs);
    Logger.recordOutput("Timers/SwerveModPd", (RobotController.getFPGATime() - initial) * 0.000001);
  }

  public void updatePIDs() {
    // io.setDriveVelPID(
    //     driveConstants.driveS,
    //     driveConstants.driveV,
    //     driveConstants.driveP,
    //     driveConstants.driveI,
    //     driveConstants.driveD);
    // io.setSteerPID(driveConstants.steerP, driveConstants.driveI, driveConstants.driveD);
    io.setDriveVelPID(driveConstants.driveS.get(), driveConstants.driveV.get(), driveConstants.driveP.get(), driveConstants.driveI.get(), driveConstants.driveD.get());
    io.setSteerPID(driveConstants.steerP.get(), driveConstants.steerI.get(), driveConstants.steerD.get());
  }

  public void setSwerveState(SwerveModuleState state) {
    Logger.recordOutput("Drive/Module" + id + "/targetSpeed", state.speedMetersPerSecond);
    Logger.recordOutput("Drive/Module" + id + "/targetAngle", state.angle);
    
    state.optimize(inputs.steerPosRelative);
    Logger.recordOutput(
        "Drive/Module" + id + "/optimizedSpeed", state.speedMetersPerSecond);
    Logger.recordOutput("Drive/Module" + id + "/optimizedAngle", state.angle);

    savedState = state;
    if (Math.abs(state.speedMetersPerSecond) < 0.1) {
      stop();
    } else {
      io.setDriveSpeed(state.speedMetersPerSecond * (driveConstants.driveMotorInversions[id-1] ? -1 : 1));

      double setpoint =
          inputs.steerPosRaw + state.angle.minus(inputs.steerPosRelative).getRotations();
      io.setSteerPos(setpoint);
    }
  }

  // returns the current state of the module as measured by the encoders
  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocityMPS, inputs.steerPosRelative);
  }
  public SwerveModulePosition getPosition() {
    double distance = inputs.drivePosition * Math.PI * 2 * driveConstants.wheelRadius;
    return new SwerveModulePosition(distance, inputs.steerPosRelative);
  }

  // returns the target states sent to the swerve module
  public SwerveModuleState getTargetState() {
    return savedState;
  }

  public void stop() {
    io.setDriveDutyCycle(0.0);
    io.setSteerDutyCycle(0.0);
  }

}
