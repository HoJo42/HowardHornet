package frc.robot.Subsystem;

import java.util.function.IntSupplier;

import SOTAlib.Encoder.Absolute.SOTA_AbsoulteEncoder;
import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.Configs.SwerveModuleConfig;

public class SwerveModule extends SubsystemBase {

  private String moduleName;
  private Double measuredDistanceMeters;
  private Double previousDistance;

  private SOTA_MotorController speedMotor;
  private PIDController speedPID;
  private SimpleMotorFeedforward speedFF;

  private SOTA_MotorController angleMotor;
  private SOTA_AbsoulteEncoder angleEncoder;
  private PIDController anglePID;
  private double angleFF;

  private double kWheelCircumference;
  private double[] gearRatio = new double[2];
  private double[] maxSpeeds = new double[2];
  private int currentGear; // 0 low, 1 high
  private IntSupplier gearSupplier;
  private final double kRotationsToRadians = 6.283185;
  private final double kRadiansToRotations = 0.159155;

  private ShuffleboardTab sTab;
  private GenericEntry liveRotationPosEntr;
  private GenericEntry rawEncoderPosEntr;
  private GenericEntry liveSpeedEntry;
  private GenericEntry angleEntry;
  private GenericEntry speedEntry;

  private double positionTolerance = 0.05;

  public SwerveModule(SOTA_MotorController speedMotor,
      SOTA_MotorController rotationMotor,
      SwerveModuleConfig config,
      SOTA_AbsoulteEncoder encoder,
      IntSupplier gearSupplier) {

    this.measuredDistanceMeters = 0.0;
    this.previousDistance = 0.0;

    this.sTab = Shuffleboard.getTab("Swerve");
    this.moduleName = config.getName();

    this.kWheelCircumference = config.getWheelCircumference();
    this.gearRatio[0] = config.getLowGearRatio();
    this.gearRatio[1] = config.getHighGearRatio();
    this.maxSpeeds[0] = config.getLowGearMaxSpeed();
    this.maxSpeeds[1] = config.getHighGearMaxSpeed();
    this.currentGear = 0;
    this.gearSupplier = gearSupplier;
    updateGear();

    this.speedMotor = speedMotor;
    this.speedPID = new PIDController(config.getSpeedP(), config.getSpeedI(), config.getSpeedD());

    this.speedFF = new SimpleMotorFeedforward(config.getSpeedS(), config.getSpeedV());

    this.angleMotor = rotationMotor;
    this.anglePID = new PIDController(config.getAngleP(), config.getAngleI(), config.getAngleD());
    anglePID.enableContinuousInput(0, 1);
    anglePID.setTolerance(positionTolerance);

    this.angleEncoder = encoder;
    this.angleFF = config.getAngleFF();

    this.rawEncoderPosEntr = sTab.add("Raw Encoder Output:" + moduleName, 0.0).getEntry();
    this.angleEntry = sTab.add("Requested Angle: " + moduleName, 0.0).getEntry();
    this.speedEntry = sTab.add("Requested Speed: " + moduleName, 0.0).getEntry();
    this.liveSpeedEntry = sTab.add("Actual speed: " + moduleName, 0.0).getEntry();
    this.liveRotationPosEntr = sTab.add("Constrained Pos: " + moduleName, 0.0).getEntry();
  }

  public void setModule(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getRotation2d());

    angleEntry.setDouble(radsToRotations(state.angle.getRadians()));
    speedEntry.setDouble(state.speedMetersPerSecond);

    double angleRotations = radsToRotations(state.angle.getRadians());
    double anglePIDOutput = anglePID.calculate(angleEncoder.getConstrainedPositon(), angleRotations);
    double angleFFOutput = angleFF * Math.signum(anglePIDOutput);

    double speedRPM = metersPerSecondToRPM(state.speedMetersPerSecond);
    double speedPIDOutput = speedPID.calculate(speedMotor.getEncoderVelocity(), speedRPM);
    double speedFFOutput = speedFF.calculate(speedRPM);

    angleMotor.setVoltage(state.speedMetersPerSecond == 0 ? 0 : anglePIDOutput + angleFFOutput);
    speedMotor.setVoltage(metersPerSecondToRPM(state.speedMetersPerSecond)); //TThis is bad
  }

  private Rotation2d getRotation2d() {
    return new Rotation2d(getRadians());
  }

  private double getRadians() {
    return angleEncoder.getConstrainedPositon() * kRotationsToRadians;
  }

  private double radsToRotations(double rads) {
    return rads * kRadiansToRotations;
  }

  private double metersPerSecondToRPM(double MPS) {
    return (60 / ((2 * Math.PI) * (kWheelCircumference / 2))) * MPS * gearRatio[currentGear];
  }

  public double getCurrentMaxSpeed() {
    return maxSpeeds[currentGear];
  }

  public Double getDistance() {
    return measuredDistanceMeters;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDistance(), getRotation2d());
  }

  public void updateGear() {
    this.currentGear = gearSupplier.getAsInt();
  }

  public void updateDistance() {
    measuredDistanceMeters += (speedMotor.getEncoderPosition() - previousDistance)
        * (kWheelCircumference / gearRatio[currentGear]);
    previousDistance = speedMotor.getEncoderPosition();
  }

  public void updateSB() {
    rawEncoderPosEntr.setDouble(angleEncoder.getRawPosition());
    liveRotationPosEntr.setDouble(angleEncoder.getConstrainedPositon());
    liveSpeedEntry.setDouble(speedMotor.getEncoderVelocity());
  }

  @Override
  public void periodic() {
    updateDistance();
    updateSB();
  }

  @Override
  public void simulationPeriodic() {
    updateSB();
  }

}
