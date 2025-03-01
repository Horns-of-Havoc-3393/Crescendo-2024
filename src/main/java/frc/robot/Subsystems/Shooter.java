package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Shooter extends SubsystemBase {
  ShooterIOSparks io;
  ShooterIOInAutoLogged inputs;

  public static class angleSetpoints {
    public static Rotation2d SHOOT = Rotation2d.fromDegrees(60);
    public static Rotation2d LOAD = Rotation2d.fromDegrees(50);
    public static Rotation2d AMP = Rotation2d.fromDegrees(110);
    public static Rotation2d DRIVE = Rotation2d.fromDegrees(0);
  }

  private LoggedDashboardNumber shooterP;
  private LoggedDashboardNumber shooterI;
  private LoggedDashboardNumber shooterD;
  private LoggedDashboardNumber shooterFF;

  private LoggedDashboardNumber elevatorP;
  private LoggedDashboardNumber elevatorI;
  private LoggedDashboardNumber elevatorD;
  private LoggedDashboardNumber elevatorG;

  private LoggedDashboardNumber conveyorP;
  private LoggedDashboardNumber conveyorI;
  private LoggedDashboardNumber conveyorD;
  private LoggedDashboardNumber conveyorFF;

  private Rotation2d angleSetpoint;
  private double speedSetpoint;

  private int encoderUpdates = 0;

  public Shooter(
      CANSparkFlex topMotor,
      CANSparkFlex bottomMotor,
      CANSparkFlex conveyorMotor,
      CANSparkMax leftElevator,
      CANSparkMax rightElevator,
      CANSparkMax intake) {

    io =
        new ShooterIOSparks(
            topMotor, bottomMotor, conveyorMotor, leftElevator, rightElevator, intake);
    inputs = new ShooterIOInAutoLogged();

    shooterP = new LoggedDashboardNumber("Shooter/shooterP");
    shooterP.set(shooterConstants.shooterP);
    shooterI = new LoggedDashboardNumber("Shooter/shooterI");
    shooterI.set(shooterConstants.shooterI);
    shooterD = new LoggedDashboardNumber("Shooter/shooterD");
    shooterD.set(shooterConstants.shooterD);
    shooterFF = new LoggedDashboardNumber("Shooter/shooterFF");
    shooterFF.set(shooterConstants.shooterFF);
    elevatorP = new LoggedDashboardNumber("Shooter/elevatorP");
    elevatorI = new LoggedDashboardNumber("Shooter/elevatorI");
    elevatorD = new LoggedDashboardNumber("Shooter/elevatorD");
    elevatorG = new LoggedDashboardNumber("Shooter/elevatorG");
    conveyorP = new LoggedDashboardNumber("Shooter/conveyorP");
    conveyorI = new LoggedDashboardNumber("Shooter/conveyorI");
    conveyorD = new LoggedDashboardNumber("Shooter/conveyorD");
    conveyorFF = new LoggedDashboardNumber("Shooter/conveyorFF");

    angleSetpoint = new Rotation2d(0);
    speedSetpoint = 0.0;

    io.updateInputs(inputs);
    io.setElevatorOffsets(
        inputs.shooterAngleAbs.minus(Rotation2d.fromRotations(inputs.elevator1Position)),
        inputs.shooterAngleAbs.minus(Rotation2d.fromRotations(inputs.elevator1Position)));
  }

  public void periodic() {
    if (encoderUpdates < 10) {
      io.setElevatorOffsets(
          inputs.shooterAngleAbs.minus(Rotation2d.fromRotations(inputs.elevator1Position)),
          inputs.shooterAngleAbs.minus(Rotation2d.fromRotations(inputs.elevator1Position)));
    }
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    io.setShooterPID(shooterP.get(), shooterI.get(), shooterD.get(), shooterFF.get());
    io.setElevatorPID(elevatorP.get(), elevatorI.get(), elevatorD.get(), elevatorG.get());
    io.setConveyorPID(conveyorP.get(), conveyorI.get(), conveyorD.get(), conveyorFF.get());

    io.setElevatorAngle(angleSetpoint);
    io.setShooterSpeed(speedSetpoint, -speedSetpoint);
  }

  public void setAngle(Rotation2d angle) {
    angleSetpoint = angle;
  }

  public void setSpeed(double speedMPS) {
    speedSetpoint = speedMPS;
  }

  public void setConveyorSpeed(double speed) {
    io.setConveyorSpeed(speed);
  }
}
