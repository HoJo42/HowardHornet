// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystem.SwerveDrive;

public class DriveCommand extends CommandBase {

  private DoubleSupplier forwardSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private SwerveDrive mSwerveDrive;

  /** Creates a new DriveCommand. */
  public DriveCommand(DoubleSupplier fwdSup, DoubleSupplier strfSup, DoubleSupplier rotatSup, SwerveDrive swerveDrive) {
    this.forwardSup = fwdSup;
    this.strafeSup = strfSup;
    this.rotationSup = rotatSup;
    this.mSwerveDrive = swerveDrive;
    addRequirements(mSwerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = Math.signum(forwardSup.getAsDouble()) * forwardSup.getAsDouble() * forwardSup.getAsDouble();
    double strafe = Math.signum(strafeSup.getAsDouble()) * strafeSup.getAsDouble() * strafeSup.getAsDouble();
    double rotation = Math.signum(rotationSup.getAsDouble()) * rotationSup.getAsDouble() * rotationSup.getAsDouble();

    mSwerveDrive.drive(forward, strafe, -rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
