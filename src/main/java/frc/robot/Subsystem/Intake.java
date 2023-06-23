// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.MotorController.SOTA_MotorController;

public class Intake extends SubsystemBase {

  private SOTA_MotorController motor;
  private DoubleSolenoid solenoid;

  //Jon what are these for?
  private Value previousValue;
  private Timer stateUpdateTimer;
  private boolean isRetracted;
  private double EXTEND_TIME = 0.8; 

  private double INTAKE_SPEED = -0.5; //TODO: move to config

  /** Creates a new Intake. */
  public Intake(SOTA_MotorController motor, DoubleSolenoid solenoid) {
    this.motor = motor;
    this.solenoid = solenoid;

    previousValue = Value.kReverse;
    stateUpdateTimer = new Timer();
    isRetracted = true;
  }

  public void intake(){
    solenoid.set(Value.kForward);
    motor.set(INTAKE_SPEED);
  }

  public void stop(){
    solenoid.set(Value.kReverse);
    motor.set(0);
  }

  public boolean isRetracted() {
    return solenoid.get() == Value.kReverse || solenoid.get() == Value.kOff ? true : false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
