// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystem.Delivery;
import frc.robot.Subsystem.Intake;

public class IntakeCommand extends ParallelCommandGroup {

  private Intake mIntake;
  private Delivery mDelivery;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake, Delivery delivery) {
    this.mIntake = intake;
    this.mDelivery = delivery;
    addCommands(new RunCommand(() -> intake.intake(), mIntake), new RunCommand(() -> delivery.intake(), mDelivery));
    
    System.out.println("New Intake Command Created");
  }
}
