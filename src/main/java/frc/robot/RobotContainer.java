// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.function.IntSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Subsystem.Delivery;
import frc.robot.Subsystem.Intake;
import frc.robot.Subsystem.Shooter;
import frc.robot.Subsystem.SwerveDrive;
import frc.robot.Subsystem.SwerveModule;
import frc.robot.Subsystem.Configs.DeliveryConfig;
import frc.robot.Subsystem.Configs.IntakeConfig;
import frc.robot.Subsystem.Configs.ShooterConfig;
import frc.robot.Subsystem.Configs.SwerveDriveConfig;
import frc.robot.Subsystem.Configs.SwerveModuleConfig;
import SOTAlib.Config.ConfigUtils;
import SOTAlib.Config.DoubleSolenoidConfig;
import SOTAlib.Config.MotorControllerConfig;
import SOTAlib.Control.SOTA_Xboxcontroller;
import SOTAlib.Factories.IllegalMotorModel;
import SOTAlib.Factories.MotorControllerFactory;
import SOTAlib.Gyro.NavX;
import SOTAlib.Gyro.SOTA_Gyro;
import SOTAlib.MotorController.SOTA_MotorController;
import SOTAlib.MotorController.SparkMaxDelegate;
import SOTAlib.Pneumatics.DoubleSolenoidShifter;

public class RobotContainer {

  private ConfigUtils mConfigUtils;

  private Delivery mDelivery;
  private Intake mIntake;
  private Shooter mShooter;
  private SwerveDrive mSwerveDrive;

  private SOTA_Xboxcontroller mController;

  public RobotContainer() {
    ObjectMapper mapper = new ObjectMapper();
    mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
    this.mConfigUtils = new ConfigUtils(mapper);

    this.mController = new SOTA_Xboxcontroller(1);

    // Intake Intiialization
    try {
      IntakeConfig intakeConfig = mConfigUtils.readFromClassPath(IntakeConfig.class, "Intake");

      MotorControllerConfig intakeMotorConfig = mConfigUtils.readFromClassPath(MotorControllerConfig.class,
          "IntakeMotor");
      SOTA_MotorController intakeMotor = MotorControllerFactory.generateMotorController(intakeMotorConfig);

      DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
      this.mIntake = new Intake(intakeMotor, intakeSolenoid, intakeConfig);
    } catch (Exception e) {
      throw new RuntimeException("Failed to create Intake.", e);
    }

    // Delivery Initialization
    try {
      DeliveryConfig deliveryConfig = mConfigUtils.readFromClassPath(DeliveryConfig.class, "Delivery");

      MotorControllerConfig deliveryMotorConfig = mConfigUtils.readFromClassPath(MotorControllerConfig.class,
          "DeliveryMotor");
      SOTA_MotorController deliveryMotor = MotorControllerFactory.generateMotorController(deliveryMotorConfig);
      this.mDelivery = new Delivery(deliveryMotor, new DigitalInput(7), new DigitalInput(0), deliveryConfig);

    } catch (Exception e) {
      throw new RuntimeException("Failed to create delivery", e);
    }

    // Shooter Initialization
    try {
      ShooterConfig shooterConfig = mConfigUtils.readFromClassPath(ShooterConfig.class, "Shooter/Config");

      MotorControllerConfig motor1Config = mConfigUtils.readFromClassPath(MotorControllerConfig.class,
          "Shooter/Motor1");
      SOTA_MotorController motor1 = MotorControllerFactory.generateMotorController(motor1Config);

      MotorControllerConfig motor2Config = mConfigUtils.readFromClassPath(MotorControllerConfig.class,
          "Shooter/Motor2");
      SOTA_MotorController motor2 = MotorControllerFactory.generateMotorController(motor2Config);

      DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5); // TODO: SOTA_Solenoid?

      mShooter = new Shooter(motor1, motor2, solenoid, shooterConfig);
    } catch (Exception e) {
      throw new RuntimeException("Failed to create Shooter", e);
    }

    //Swerve Initialization
    try {
      SOTA_Gyro swerveGyro = new NavX(new AHRS(Port.kMXP));

      DoubleSolenoidConfig shifterConfig = mConfigUtils.readFromClassPath(DoubleSolenoidConfig.class, "Swerve/Shifter");
      DoubleSolenoid swerveSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
      DoubleSolenoidShifter swerveShifter = new DoubleSolenoidShifter(swerveSolenoid, shifterConfig);

      SwerveDriveConfig swerveConfig = mConfigUtils.readFromClassPath(SwerveDriveConfig.class, "Swerve/Drive");

      SwerveModule[] swerveModules = {
          initModule("Swerve/FrontLeft/Speed", "Swerve/FrontLeft/Angle", "Swerve/FrontLeft/Module",
              swerveShifter::getGear),
          initModule("Swerve/FrontRight/Speed", "Swerve/FrontRight/Angle", "Swerve/FrontRight/Module",
              swerveShifter::getGear),
          initModule("Swerve/BackLeft/Speed", "Swerve/BackLeft/Angle", "Swerve/BackLeft/Module",
              swerveShifter::getGear),
          initModule("Swerve/BackRight/Speed", "Swerve/BackRight/Angle", "Swerve/BackRight/Module",
              swerveShifter::getGear)
      };

      this.mSwerveDrive = new SwerveDrive(swerveModules, swerveGyro, swerveShifter, swerveConfig);
    } catch (Exception e) {
      throw new RuntimeException("Failed to create Swerve", e);
    }

    configureBindings();
  }

  private void configureBindings() {
    mController.a().whileTrue(new IntakeCommand(mIntake, mDelivery));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public SwerveModule initModule(String speedConfig, String angleConfig, String moduleConfig, IntSupplier gear) {
    try {
      MotorControllerConfig spdConfig = mConfigUtils.readFromClassPath(MotorControllerConfig.class, speedConfig);
      SOTA_MotorController speedMotor = MotorControllerFactory.generateMotorController(spdConfig);
      System.out.println("Speed Correct");

      MotorControllerConfig nglConfig = mConfigUtils.readFromClassPath(MotorControllerConfig.class, angleConfig);
      SOTA_MotorController angleMotor = MotorControllerFactory.generateMotorController(nglConfig);
      System.out.println("Angle Correct");

      SwerveModuleConfig mdlConfig = mConfigUtils.readFromClassPath(SwerveModuleConfig.class, moduleConfig);
      System.out.println("modConfig Correct");
      return new SwerveModule(speedMotor, angleMotor, mdlConfig, gear);
    } catch (IllegalMotorModel e) {
      throw new RuntimeException("Failed to create module", e);
    } catch (IOException e) {
      throw new RuntimeException("Failed to create module", e);
    }
  }

}
