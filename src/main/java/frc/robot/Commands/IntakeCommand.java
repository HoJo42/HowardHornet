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

public class IntakeCommand extends CommandBase {

  private Intake mIntake;
  private Delivery mDelivery;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake, Delivery delivery) {
    this.mIntake = intake;
    this.mDelivery = delivery;
    addRequirements(mIntake, mDelivery);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("Intake Command Running!");
    new ParallelCommandGroup(
        new RunCommand(() -> mIntake.intake(), mIntake),
        new RunCommand(() -> mDelivery.intake(), mDelivery));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("Intake Command Stopped!");
    new ParallelCommandGroup(
        new RunCommand(() -> mIntake.stop(), mIntake),
        new RunCommand(() -> mDelivery.stop(), mDelivery));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
