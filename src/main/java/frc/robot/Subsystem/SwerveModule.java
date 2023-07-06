package frc.robot.Subsystem;

import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.Configs.SwerveModuleConfig;

public class SwerveModule extends SubsystemBase {

  private SOTA_MotorController speedMotor;
  private ProfiledPIDController speedPID;
  private SimpleMotorFeedforward speedFF;

  private SOTA_MotorController angleMotor;
  private ProfiledPIDController anglePID;
  private SimpleMotorFeedforward angleFF;

  private final double kRotationCountsPerRevolution;
  private double kWheelCircumference;
  private double[] gearRatio;
  private int currentGear; //0 low, 1 high
  private double kSpeedCountsPerRevolution;

  public SwerveModule(SOTA_MotorController speedMotor,
      SOTA_MotorController rotationMotor,
      SwerveModuleConfig config) {

    this.kRotationCountsPerRevolution = config.getAngleCountsPerRevolution();
    this.kSpeedCountsPerRevolution = config.getSpeedCountsPerRevolution();
    this.kWheelCircumference = config.getWheelCircumference();
    this.gearRatio[0] = config.getLowGearRatio();
    this.gearRatio[1] = config.getHighGearRatio();
    this.currentGear = 0;

    this.speedMotor = speedMotor;
    this.speedPID = new ProfiledPIDController(config.getSpeedP(), config.getSpeedI(), config.getSpeedD(),
        new Constraints(config.getSpeedMaxVelocity(), config.getSpeedMaxAcceleration()));

    this.speedFF = new SimpleMotorFeedforward(config.getSpeedS(), config.getSpeedV());

    this.angleMotor = rotationMotor;
    this.anglePID = new ProfiledPIDController(config.getAngleP(), config.getAngleI(), config.getAngleD(),
        new Constraints(config.getAngleMaxVelocity(), config.getAngleMaxAcceleration()));
    anglePID.enableContinuousInput(0, kRotationCountsPerRevolution);

    this.angleFF = new SimpleMotorFeedforward(config.getAngleS(), config.getAngleV());

  }

  public void setModule(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getRotation2d());

    double angleCounts = radsToNative(state.angle.getRadians());
    double anglePIDOutput = anglePID.calculate(angleMotor.getEncoderPosition(), angleCounts);
    double angleFFOutput = angleFF.calculate(anglePID.getSetpoint().velocity);

    double speedCounts = metersPerSecondToNative(state.speedMetersPerSecond);
    double speedPIDOutput = speedPID.calculate(speedMotor.getEncoderVelocity(), speedCounts);
    double speedFFOutput = speedFF.calculate(speedCounts);

    angleMotor.setVoltage(state.speedMetersPerSecond == 0 ? 0 : anglePIDOutput + angleFFOutput);
    speedMotor.setVoltage(speedPIDOutput + speedFFOutput);
  }

  private Rotation2d getRotation2d() {
    return new Rotation2d(getRadians());
  }

  private double getRadians() {
    return (2 * Math.PI / kRotationCountsPerRevolution) * angleMotor.getEncoderPosition();
  }

  private double radsToNative(double rads) {
    return (kRotationCountsPerRevolution / (2 * Math.PI)) * rads;
  }

  private double metersPerSecondToNative(double MPS) {
    return MPS / getMetersPerCount() / 10;
  }

  public double getMetersPerCount() {
    return kWheelCircumference / gearRatio[currentGear] / kSpeedCountsPerRevolution;
  }

  private void setCurrentGear(int gear) {
    this.currentGear = gear;
  }

}
