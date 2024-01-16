package frc.robot.Subsystem;

import java.util.function.IntSupplier;

import SOTAlib.Encoder.Absolute.SOTA_AbsoulteEncoder;
import SOTAlib.Math.Conversions;
import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private double[] gearRatio = { 0, 0 };
  private double[] maxSpeeds = { 0, 0 };
  private int currentGear; // 0 low, 1 high
  private IntSupplier gearSupplier;
  private final double kRotationsToRadians = 6.283185;
  private final double kRadiansToRotations = 0.159155;

  private ShuffleboardTab sTab;
  private GenericEntry encoderPosEntry;
  private GenericEntry angleEntry;
  private GenericEntry speedEntry;

  private SimDeviceSim simAngleEncoder;
  private SimDouble simEncoderPosition;
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

    this.encoderPosEntry = sTab.add("Encoder Output:" + moduleName, 0.0).getEntry();
    this.angleEntry = sTab.add("Requested Angle: " + moduleName, 0.0).getEntry();
    this.speedEntry = sTab.add("Requested Speed: " + moduleName, 0.0).getEntry();
  }

  public void setModule(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getRotation2d());

    angleEntry.setDouble(state.angle.getRadians());
    speedEntry.setDouble(Conversions.metersPerSecToFeetPerSec(state.speedMetersPerSecond));

    double angleRotations = radsToRotations(state.angle.getRadians());
    double anglePIDOutput = anglePID.calculate(angleEncoder.getConstrainedPositon(), angleRotations);
    double angleFFOutput = angleFF * Math.signum(anglePIDOutput);

    double speedRPM = metersPerSecondToRPM(state.speedMetersPerSecond);
    double speedPIDOutput = speedPID.calculate(speedMotor.getEncoderVelocity(), speedRPM);
    double speedFFOutput = speedFF.calculate(speedRPM);

    angleMotor.setVoltage(state.speedMetersPerSecond == 0 ? 0 : anglePIDOutput + angleFFOutput);
    // speedMotor.setVoltage(speedPIDOutput + speedFFOutput);
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
    return (60 / ((2 * Math.PI) * (kWheelCircumference / 2))) * MPS;
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
    measuredDistanceMeters += (speedMotor.getEncoderPosition() - previousDistance) * (kWheelCircumference / gearRatio[currentGear]);
    previousDistance = speedMotor.getEncoderPosition();
  }

  public void updateSB() {
    encoderPosEntry.setDouble(angleEncoder.getConstrainedPositon());
    SmartDashboard.putNumber("Angle Encoder nooff" + moduleName, angleEncoder.getRawPosition());
    SmartDashboard.putNumber("Constrained Pos: " + moduleName, angleEncoder.getConstrainedPositon());
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
