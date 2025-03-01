package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.SwerveAbs;
import frc.robot.Commands.autoCmd;
import frc.robot.Constants.driveConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.Positioning.PosIONavX;
import frc.robot.Subsystems.Drive.SwerveBase;
import frc.robot.Subsystems.Shooter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class RobotContainer {

  TalonFX[] driveMotors = new TalonFX[4];
  TalonFX[] steerMotors = new TalonFX[4];
  CANcoder[] encoders = new CANcoder[4];
  CANSparkMax elevator1 = new CANSparkMax(20, MotorType.kBrushless);
  CANSparkMax elevator2 = new CANSparkMax(21, MotorType.kBrushless);
  CANSparkMax intake = new CANSparkMax(22, MotorType.kBrushless);
  CANSparkFlex shooter1 = new CANSparkFlex(24, MotorType.kBrushless);
  CANSparkFlex shooter2 = new CANSparkFlex(23, MotorType.kBrushless);
  CANSparkFlex conveyor = new CANSparkFlex(25, MotorType.kBrushless);
  public SwerveBase swerve;
  public Shooter shooter;

  LoggedDashboardNumber shooterSpeed = new LoggedDashboardNumber("Shooter/speed", 0.0);

  public SwerveAbs absCmd;

  private final CommandXboxController controller = new CommandXboxController(0);

  public autoCmd auto;

  long spinUp;

  public RobotContainer() {
    deviceFactory();

    swerve =
        new SwerveBase(
            driveMotors,
            steerMotors,
            encoders,
            driveConstants.offsets,
            driveConstants.absoluteEncoderOffsets,
            new PosIONavX(new AHRS()));

    shooter = new Shooter(shooter1, shooter2, conveyor, elevator1, elevator2, intake);
    configureBinds();

    // auto = new autoCmd(swerve);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable pids = inst.getTable("SmartDashboard/PIDs");
    pids.getDoubleTopic("driveS").publish().set(driveConstants.driveS);
    pids.getDoubleTopic("driveV").publish().set(driveConstants.driveV);
    pids.getDoubleTopic("driveP").publish().set(driveConstants.driveP);
    pids.getDoubleTopic("driveI").publish().set(driveConstants.driveI);
    pids.getDoubleTopic("driveD").publish().set(driveConstants.driveD);

    pids.getDoubleTopic("steerP").publish().set(driveConstants.steerP);
    pids.getDoubleTopic("steerI").publish().set(driveConstants.steerI);
    pids.getDoubleTopic("steerD").publish().set(driveConstants.steerD);

    absCmd = new SwerveAbs(swerve, controller);

    shooterSpeed.set(shooterConstants.shooterSpeed);
  }

  private void configureBinds() {
    controller
        .rightStick()
        .onTrue(
            Commands.runOnce(
                () -> {
                  swerve.zeroGyro();
                  System.out.println("zero");
                }));
    controller.y().onTrue(Commands.runOnce(() -> shooter.setAngle(Shooter.angleSetpoints.SHOOT)));
    controller.b().onTrue(Commands.runOnce(() -> shooter.setAngle(Shooter.angleSetpoints.DRIVE)));
    controller.x().onTrue(Commands.runOnce(() -> shooter.setAngle(Shooter.angleSetpoints.AMP)));
    controller.a().onTrue(Commands.runOnce(() -> shooter.setAngle(Shooter.angleSetpoints.LOAD)));

    controller
        .leftTrigger(0.5)
        .whileTrue(
            Commands.run(
                    () -> {
                      shooter.setConveyorSpeed(0.5);
                      shooter.setSpeed(-5);
                      System.out.println("trigger");
                    })
                .finallyDo(
                    () -> {
                      shooter.setConveyorSpeed(0);
                      shooter.setSpeed(0);
                    }));
    controller
        .rightTrigger(0.5)
        .whileTrue(
            Commands.run(
                    () -> {
                      if ((Logger.getRealTimestamp() - spinUp) * 0.000001 > 2) {
                        shooter.setConveyorSpeed(-3);
                        System.out.println("trigger2");
                      }
                    })
                .beforeStarting(
                    Commands.runOnce(
                        () -> {
                          shooter.setSpeed(shooterSpeed.get());
                          spinUp = Logger.getRealTimestamp();
                        }))
                .finallyDo(
                    () -> {
                      shooter.setConveyorSpeed(0);
                      shooter.setSpeed(0);
                    }));
    controller
        .rightBumper()
        .whileTrue(
            Commands.run(
                    () -> {
                      shooter.setSpeed(15);
                      System.out.println("bumper");
                    })
                .finallyDo(() -> shooter.setSpeed(0)));
  }

  private void deviceFactory() {
    for (int i = 0; i < 4; i++) {
      driveMotors[i] = new TalonFX(i + 1);
      steerMotors[i] = new TalonFX(i + 5);
      encoders[i] = new CANcoder(i + 9);
    }
  }
}
