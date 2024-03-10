package frc.robot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.driveConstants;
import frc.robot.Subsystems.Drive.SwerveBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class SwerveAbs extends Command {

  SwerveBase swerve;

  CommandXboxController controller;

  LoggedDashboardNumber deadzone;
  LoggedDashboardNumber lateralMaxSpeed;
  LoggedDashboardNumber rotationalMaxSpeed;

  SlewRateLimiter xLimit;
  SlewRateLimiter yLimit;
  SlewRateLimiter rLimit;

  LoggedDashboardNumber latAccLimit;
  LoggedDashboardNumber rotAccLimit;
  LoggedDashboardBoolean update;

  public SwerveAbs(SwerveBase swerve, CommandXboxController controller) {
    this.controller = controller;
    this.swerve = swerve;

    this.addRequirements(swerve);
    System.out.println("initCMD");

    deadzone = new LoggedDashboardNumber("Control/deadzone");
    lateralMaxSpeed = new LoggedDashboardNumber("Control/lateralMaxSpeed");
    rotationalMaxSpeed = new LoggedDashboardNumber("Control/rotationalMaxSpeed");
    latAccLimit = new LoggedDashboardNumber("Control/LateralAcceleration");
    rotAccLimit = new LoggedDashboardNumber("Control/RotationalAcceleration");
    update = new LoggedDashboardBoolean("update");

    xLimit = new SlewRateLimiter(driveConstants.lateralAccelLimitMPSPS);
    yLimit = new SlewRateLimiter(driveConstants.lateralAccelLimitMPSPS);
    rLimit = new SlewRateLimiter(driveConstants.lateralAccelLimitMPSPS);
  }

  public void execute() {
    double initial = Logger.getRealTimestamp();
    Logger.recordOutput("Drive/AbsCMD/xAxis", controller.getLeftY());
    Logger.recordOutput("Drive/AbsCMD/yAxis", controller.getLeftX());
    Logger.recordOutput("Drive/AbsCMD/betaAxis", controller.getRightX());
    if (update.get()) {
      xLimit = new SlewRateLimiter(driveConstants.lateralAccelLimitMPSPS);
      yLimit = new SlewRateLimiter(driveConstants.lateralAccelLimitMPSPS);
      rLimit = new SlewRateLimiter(driveConstants.rotationalAccelLimitRPSPS);
    }
    if ((Math.pow(controller.getLeftY(), 2)
            + Math.pow(controller.getLeftX(), 2)
            + Math.pow(controller.getRightX(), 2))
        > driveConstants.deadZone) {
      swerve.setFO(
          new ChassisSpeeds(
              xLimit.calculate(controller.getLeftY() * driveConstants.maxSpeedMPS),
              yLimit.calculate(controller.getLeftX() * driveConstants.maxSpeedMPS),
              rLimit.calculate(controller.getRightX() * -1 * driveConstants.maxRotRPS)),
          driveConstants.maxSpeedMPS);
    } else {
      swerve.setFO(
          new ChassisSpeeds(xLimit.calculate(0), yLimit.calculate(0), rLimit.calculate(0)),
          driveConstants.maxSpeedMPS);
    }
    Logger.recordOutput("Timers/SwerveAbsEx", (Logger.getRealTimestamp() - initial) * 0.000001);
  }
}
