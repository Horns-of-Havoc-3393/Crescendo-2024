package frc.robot.Positioning;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PosIONavX implements PosIO {

  AHRS navx;
 

  DoubleSubscriber estXSub;
  DoubleSubscriber estYSub;
  DoubleSubscriber estZSub;
  DoubleSubscriber kGainSub;

  DoublePublisher xAccelPub;
  DoublePublisher yAccelPub;
  DoublePublisher zAccelPub;

  public PosIONavX(AHRS navx) {
    // this.navx = navx;
    this.navx = new AHRS(NavXComType.kMXP_SPI);

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
    inputs.zGyro = Rotation2d.fromDegrees(-navx.getYaw());

    inputs.xAccel = 0.0; // navx.getRawAccelX();
    inputs.yAccel = 0.0; // navx.getRawAccelY();
    inputs.zAccel = 0.0; // navx.getRawAccelZ();

    inputs.kGain = kGainSub.get();
    inputs.estX = estXSub.get();
    inputs.estY = estYSub.get();
    inputs.estZ = estZSub.get();
  }

  public void zero() {
    navx.reset();
  }
}
