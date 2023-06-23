// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.Configs.DeliveryConfig;
import lib.MotorController.SOTA_MotorController;

public class Delivery extends SubsystemBase {

  private SOTA_MotorController motor;

  private DigitalInput topSensor;
  private DigitalInput bottomSensor;

  private double INTAKE_SPEED; //TODO: add config
  private double OUTTAKE_SPEED;
  private double SHOOT_SPEED;

  /** Creates a new Delivery. */
  public Delivery(SOTA_MotorController motor, DigitalInput topSensor, DigitalInput bottomSensor, DeliveryConfig config) {
    this.motor = motor;
    this.topSensor = topSensor;
    this.bottomSensor = bottomSensor;

    this.INTAKE_SPEED = config.getIntakeSpeed();
    this.OUTTAKE_SPEED = config.getOutTakeSpeed();
    this.SHOOT_SPEED = config.getShootSpeed();
  }

  /**
   * Runs delivery towards flywheel for intake
   */
  public void intake(){
    motor.set(INTAKE_SPEED);
  }

  /**
   * Runs delivery away from flywheel
   */
  public void outTake(){
    motor.set(OUTTAKE_SPEED);
  }

  /**
   * Runs delivery towards flywheel for shooting
   */
  public void shoot(){
    motor.set(SHOOT_SPEED);
  }

  /**
   * 
   *  @return State of top sensor
   */
  public boolean getTopSensor(){
    return topSensor.get();
  }

  /**
   * 
   * @return State of bottom sensor
   */
  public boolean getBottomSensor(){
    return bottomSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
