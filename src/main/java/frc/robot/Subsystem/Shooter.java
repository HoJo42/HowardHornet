// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import java.util.ArrayList;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.Configs.ShooterConfig;

public class Shooter extends SubsystemBase {

  private DoubleSolenoid hoodSolenoid;
  private SOTA_MotorController motor1;
  private SOTA_MotorController motor2;
  
  private SimpleMotorFeedforward feedforward;
  private PIDController speedPID;

  private ArrayList<Double> speedSample;

  private ShuffleboardTab tab;
  private GenericEntry shooterRPM;
  private double defaultRPM;

  /** Creates a new Shooter. */
  public Shooter(SOTA_MotorController motor1, SOTA_MotorController motor2, DoubleSolenoid hoodSolenoid, ShooterConfig config) {
    this.motor1 = motor1;
    this.motor2 = motor2;
    this.hoodSolenoid = hoodSolenoid;

    this.feedforward = new SimpleMotorFeedforward(config.getKS(), config.getKV());
    this.speedPID = new PIDController(config.getKP(), config.getKI(), config.getKD());

    this.defaultRPM = config.getDefaultRPM();
    this.tab = Shuffleboard.getTab("Shooter");
    this.shooterRPM = tab.addPersistent("ShooterRPM", defaultRPM).getEntry();

    this.speedSample = new ArrayList<>();
    speedSample.add(0.0);
  }

  public void stopFlywheel() {
    motor1.stopMotor();
    motor2.stopMotor();
  }

  /**
   * Checks if it is ready to shoot with the average speed of the flywheel
   * @return Whether or not it is ready to shoot
   */
  public boolean readyToShoot() {
    double averageSpeed = 0.0;
    for (int i = 0; i < speedSample.size(); i++) {
      averageSpeed += speedSample.get(i);
    }
    averageSpeed /= speedSample.size();

    tab.add("live shooter RPM", averageSpeed);

    return Math.abs(averageSpeed - speedPID.getSetpoint()) < 100;
  }

  

  /**
   * Sets the RPM of the motor with PID and FF
   * @param rpm Setpoint for the motor's RPM
   */
  public void setMotorRPM(double rpm) {
    rpm = shooterRPM.getDouble(defaultRPM);
     double motorInput = feedforward.calculate(rpm) + speedPID.calculate(getMotorRPM(), rpm);
     motor1.setVoltage(motorInput);
     motor2.setVoltage(motorInput);
     speedPID.setSetpoint(rpm);
   }

   public void initializeShootingSequence() {
    double motorInput = feedforward.calculate(shooterRPM.getDouble(defaultRPM)) + speedPID.calculate(getMotorRPM(), shooterRPM.getDouble(defaultRPM));
    motor1.setVoltage(motorInput);
    motor2.setVoltage(motorInput);
    speedPID.setSetpoint(shooterRPM.getDouble(defaultRPM));
  }

   /**
   * Returns the motor RPM
   * @return the motors RPM, not the flywheels
   */
  public double getMotorRPM(){
    return Math.abs(motor1.getEncoder().getVelocity());
  }

  /**
   * Raises the hood up
   */
  public void hoodUp() {
    hoodSolenoid.set(Value.kReverse);
  }

  /**
   * Lowers the hood
   */
  public void hoodDown() {
    hoodSolenoid.set(Value.kForward);
  }

  /**
   * Updates the speed sample for the flywheel speed to be used in {@link #readyToShoot()}
   */
  private void updateSpeedSample() {
    speedSample.add(getMotorRPM());
    if (speedSample.size() > 10) {
      speedSample.remove(0);
    }
  }

  @Override
  public void periodic() {
    updateSpeedSample();
  }
}
