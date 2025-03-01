package frc.robot.Positioning;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class PosIONavX implements PosIO {

  // AHRS navx;
  ADXRS450_Gyro gyro;

  DoubleSubscriber estXSub;
  DoubleSubscriber estYSub;
  DoubleSubscriber estZSub;
  DoubleSubscriber kGainSub;

  DoublePublisher xAccelPub;
  DoublePublisher yAccelPub;
  DoublePublisher zAccelPub;

  public PosIONavX(AHRS navx) {
    // this.navx = navx;
    this.gyro = new ADXRS450_Gyro();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable kalmanT = inst.getTable("/Kalman");
    estXSub = kalmanT.getDoubleTopic("estX").subscribe(0.0);
    estYSub = kalmanT.getDoubleTopic("estY").subscribe(0.0);
    estZSub = kalmanT.getDoubleTopic("estZ").subscribe(0.0);
    kGainSub = kalmanT.getDoubleTopic("kGain").subscribe(0.0);

    xAccelPub = kalmanT.getDoubleTopic("xAccel").publish();
    yAccelPub = kalmanT.getDoubleTopic("yAccel").publish();
    zAccelPub = kalmanT.getDoubleTopic("zAccel").publish();
  }

  @Override
  public void updateInputs(PosIOIn inputs) {
    inputs.xGyro = Rotation2d.fromDegrees(0);
    inputs.yGyro = Rotation2d.fromDegrees(0);
    inputs.zGyro = Rotation2d.fromDegrees(gyro.getAngle() * -1);

    inputs.xAccel = 0.0; // navx.getRawAccelX();
    inputs.yAccel = 0.0; // navx.getRawAccelY();
    inputs.zAccel = 0.0; // navx.getRawAccelZ();

    inputs.kGain = kGainSub.get();
    inputs.estX = estXSub.get();
    inputs.estY = estYSub.get();
    inputs.estZ = estZSub.get();
  }

  public void zero() {
    gyro.reset();
  }
}
