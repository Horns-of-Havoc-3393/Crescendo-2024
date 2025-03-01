package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.SwerveAbs;
import frc.robot.Commands.autoCmd;
import frc.robot.Constants.driveConstants;
import frc.robot.Positioning.PosIONavX;
import frc.robot.Subsystems.Drive.SwerveBase;
import frc.robot.Subsystems.ElvManipSubsystem;

public class RobotContainer {

  TalonFX[] driveMotors = new TalonFX[4];
  TalonFX[] steerMotors = new TalonFX[4];
  CANcoder[] encoders = new CANcoder[4];
  SparkMax elevator1 = new SparkMax(21, MotorType.kBrushless);
  SparkMax elevator2 = new SparkMax(22, MotorType.kBrushless);
  SparkMax wristMotor = new SparkMax(31, MotorType.kBrushless);
  SparkMax rollerMotor = new SparkMax(32, MotorType.kBrushless);
  public SwerveBase swerve;
  public ElvManipSubsystem manipulator;

  public ElvManipSubsystem elvManSub = new ElvManipSubsystem(elevator1, elevator2, wristMotor, rollerMotor);

  LoggedNetworkNumber shooterSpeed = new LoggedNetworkNumber("/SmartDashboard/Shooter/speed", 0.0);

  public SwerveAbs absCmd;

  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController armOperater = new CommandXboxController(1);

  public autoCmd auto;

  long spinUp;

  public RobotContainer() {
    deviceFactory();

    Constants.initLiveConstants();
    swerve =
        new SwerveBase(
            driveMotors,
            steerMotors,
            encoders,
            driveConstants.offsets,
            driveConstants.absoluteEncoderOffsets,
            new PosIONavX(new AHRS(AHRS.NavXComType.kMXP_SPI)));

    configureBinds();

    // auto = new autoCmd(swerve);

    absCmd = new SwerveAbs(swerve, controller);

  }

  private void configureBinds() {
    controller.y().onTrue(new InstantCommand(() -> {swerve.zeroGyro();}, swerve));

    armOperater.rightBumper().whileTrue(new InstantCommand(() -> {manipulator.normal_out();}, manipulator));
    armOperater.leftBumper().whileTrue(new InstantCommand(() -> {manipulator.normal_in();}, manipulator));
    armOperater.start().onTrue(new InstantCommand(() -> {manipulator.gotoSetpoint(ElvManipSubsystem.setpoints.CORAL);}, manipulator));
    armOperater.start().onFalse(new InstantCommand(() -> {manipulator.gotoSetpoint(ElvManipSubsystem.setpoints.STOW);}, manipulator));
    armOperater.a().onTrue(new InstantCommand(() -> {manipulator.gotoSetpoint(ElvManipSubsystem.setpoints.L1);}, manipulator));
    armOperater.x().onTrue(new InstantCommand(() -> {manipulator.gotoSetpoint(ElvManipSubsystem.setpoints.L2);}, manipulator));
    armOperater.y().onTrue(new InstantCommand(() -> {manipulator.gotoSetpoint(ElvManipSubsystem.setpoints.L3);}, manipulator));
    armOperater.b().onTrue(new InstantCommand(() -> {manipulator.gotoSetpoint(ElvManipSubsystem.setpoints.L4);}, manipulator));
    armOperater.back().onTrue(new InstantCommand(() -> {manipulator.gotoSetpoint(ElvManipSubsystem.setpoints.DISLODGE);}, manipulator));

    //swerve.setDefaultCommand(absCmd);
  }

  private void deviceFactory() {
    for (int i = 0; i < 4; i++) {
      driveMotors[i] = new TalonFX(i + 1);
      steerMotors[i] = new TalonFX(i + 5);
      encoders[i] = new CANcoder(i + 9);
    }
  }
}
