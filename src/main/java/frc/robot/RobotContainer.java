// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystem.Delivery;
import frc.robot.Subsystem.Configs.DeliveryConfig;
import lib.Config.ConfigUtils;
import lib.Config.MotorControllerConfig;
import lib.Factories.MotorControllerFactory;
import lib.MotorController.SOTA_MotorController;
import lib.MotorController.SparkMaxDelegate;

public class RobotContainer {

  private ConfigUtils mConfigUtils;

  private Delivery mDelivery;

  public RobotContainer() {
    ObjectMapper mapper = new ObjectMapper();
    mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
    this.mConfigUtils = new ConfigUtils(mapper);

    try {
      DeliveryConfig deliveryConfig = mConfigUtils.readFromClassPath(DeliveryConfig.class, "Delivery");

      MotorControllerConfig deliveryMotorConfig = mConfigUtils.readFromClassPath(MotorControllerConfig.class, "DeliveryMotor");
      SOTA_MotorController deliveryMotor = MotorControllerFactory.generateTalon(deliveryMotorConfig);
      this.mDelivery = new Delivery(deliveryMotor, new DigitalInput(7), new DigitalInput(0), deliveryConfig);
      
    } catch (Exception e) {
      throw new RuntimeException("Failed to create delivery", e);
    }
    
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
