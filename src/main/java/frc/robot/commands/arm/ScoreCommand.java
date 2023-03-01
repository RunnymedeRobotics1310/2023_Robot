// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.operator.DriverController;
import frc.robot.subsystems.ArmSubsystem;

public class ScoreCommand extends CommandBase {

  private final ArmSubsystem     armSubsystem;
  private final DriverController driverController;

  /**
   * Creates a new ScoreCommand.
   * 
   * @param ArmSubsystem The subsystem used by this command.
   */
  public ScoreCommand(DriverController driverController, ArmSubsystem armSubsystem) {
    this.driverController = driverController;
    this.armSubsystem     = armSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driverController.isLow()) {

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
