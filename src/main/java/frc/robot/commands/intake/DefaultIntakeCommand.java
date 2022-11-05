package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final XboxController driverController;

    /**
     * Creates a new ExampleCommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public DefaultIntakeCommand(XboxController driverController, IntakeSubsystem intakeSubsystem) {

        this.driverController = driverController;
        this.intakeSubsystem = intakeSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    // @Override
    // public void execute() {

    // if (driverController.getRawButton(1) == true) {
    // intakeSubsystem.setMotorSpeed(0.5);
    // intakeSubsystem.setRollerPiston(true);
    // intakeSubsystem.setHoodPiston(true);
    // }
    // else {
    // intakeSubsystem.setMotorSpeed(0);
    // intakeSubsystem.setRollerPiston(false);
    // intakeSubsystem.setHoodPiston(false);
    // }
    // }

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
