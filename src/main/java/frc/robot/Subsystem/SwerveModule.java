package frc.robot.Subsystem;

import java.lang.annotation.Native;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.Configs.SwerveModuleConfig;

public class SwerveModule extends SubsystemBase {

  private SOTA_MotorController speedMotor;
  private ProfiledPIDController speedPID;

  private SOTA_MotorController angleMotor;
  private ProfiledPIDController anglePID;
  private SimpleMotorFeedforward angleFF;

  private final double kRotationCountsPerRevolution;

  public SwerveModule(SOTA_MotorController speedMotor,
      SOTA_MotorController rotationMotor,
      SwerveModuleConfig config) {

    this.speedMotor = speedMotor;
    this.speedPID = new ProfiledPIDController(config.getSpeedP(), config.getSpeedI(), config.getSpeedD(),
        new Constraints(config.getSpeedMaxVelocity(), config.getSpeedMaxAcceleration()));

    this.angleMotor = rotationMotor;
    this.anglePID = new ProfiledPIDController(config.getAngleP(), config.getAngleI(), config.getAngleD(),
        new Constraints(config.getAngleMaxVelocity(), config.getAngleMaxAcceleration()));
    this.angleFF = new SimpleMotorFeedforward(config.getAngleS(), config.getAngleV());

    this.kRotationCountsPerRevolution = config.getAngleCountsPerRevolution();
  }

  public void setModule(SwerveModuleState state){
    state = SwerveModuleState.optimize(state, getRotation2d());

    double angleCounts = radsToNative(state.angle.getRadians());
    double anglePIDOutput = anglePID.calculate(angleMotor.getEncoderPosition(), angleCounts);
    double angleFFOutput = angleFF.calculate(anglePID.getSetpoint().velocity);

    

    angleMotor.setVoltage(state.speedMetersPerSecond == 0 ? 0 : anglePIDOutput + angleFFOutput);
  }

  private Rotation2d getRotation2d() {
    return new Rotation2d(getRadians());
  }

  private double getRadians() {
    return ( 2 * Math.PI / kRotationCountsPerRevolution) * angleMotor.getEncoderPosition();
  }

  private double radsToNative(double rads) {
    return (kRotationCountsPerRevolution / (2 * Math.PI)) * rads;
  }

}
