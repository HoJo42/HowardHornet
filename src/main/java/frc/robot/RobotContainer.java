// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystem.Delivery;
import frc.robot.Subsystem.Intake;
import frc.robot.Subsystem.Shooter;
import frc.robot.Subsystem.Configs.DeliveryConfig;
import frc.robot.Subsystem.Configs.IntakeConfig;
import frc.robot.Subsystem.Configs.ShooterConfig;
import SOTAlib.Config.ConfigUtils;
import SOTAlib.Config.MotorControllerConfig;
import SOTAlib.Factories.MotorControllerFactory;
import SOTAlib.MotorController.SOTA_MotorController;
import SOTAlib.MotorController.SparkMaxDelegate;

public class RobotContainer {

  private ConfigUtils mConfigUtils;

  private Delivery mDelivery;
  private Intake mIntake;
  private Shooter mShooter;

  public RobotContainer() {
    ObjectMapper mapper = new ObjectMapper();
    mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
    this.mConfigUtils = new ConfigUtils(mapper);

    //Intake Intiialization
    try {
      IntakeConfig intakeConfig = mConfigUtils.readFromClassPath(IntakeConfig.class, "Intake");

      MotorControllerConfig intakeMotorConfig = mConfigUtils.readFromClassPath(MotorControllerConfig.class, "IntakeMotor");
      SOTA_MotorController intakeMotor = MotorControllerFactory.generateMotorController(intakeMotorConfig);
      DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
      
      this.mIntake = new Intake(intakeMotor, intakeSolenoid, intakeConfig);
    } catch (Exception e) {
      throw new RuntimeException("Failed to create Intake.", e);
    }

    //Delivery Initialization
    try {
      DeliveryConfig deliveryConfig = mConfigUtils.readFromClassPath(DeliveryConfig.class, "Delivery");

      MotorControllerConfig deliveryMotorConfig = mConfigUtils.readFromClassPath(MotorControllerConfig.class, "DeliveryMotor");
      SOTA_MotorController deliveryMotor = MotorControllerFactory.generateMotorController(deliveryMotorConfig);
      this.mDelivery = new Delivery(deliveryMotor, new DigitalInput(7), new DigitalInput(0), deliveryConfig);
      
    } catch (Exception e) {
      throw new RuntimeException("Failed to create delivery", e);
    }

    //Shooter Initialization
    try {
      ShooterConfig shooterConfig = mConfigUtils.readFromClassPath(ShooterConfig.class, "Shooter/Config");

      MotorControllerConfig motor1Config = mConfigUtils.readFromClassPath(MotorControllerConfig.class, "Shooter/Motor1");
      SOTA_MotorController motor1 = MotorControllerFactory.generateMotorController(motor1Config);

      MotorControllerConfig motor2Config = mConfigUtils.readFromClassPath(MotorControllerConfig.class, "Shooter/Motor2");
      SOTA_MotorController motor2 = MotorControllerFactory.generateMotorController(motor2Config);
      
      DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5); //TODO: SOTA_Solenoid?

      mShooter = new Shooter(motor1, motor2, solenoid, shooterConfig);
    } catch (Exception e) {
      throw new RuntimeException("Failed to create Shooter", e);
    }
    
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
