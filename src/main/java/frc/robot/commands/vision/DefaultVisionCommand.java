package frc.robot.commands.vision;

import frc.robot.commands.RunnymedeCommandBase;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.VisionSubsystem;

public class DefaultVisionCommand extends RunnymedeCommandBase {

    private final VisionSubsystem visionSubsystem;
    private final OperatorInput   driverController;

    /**
     * Default Vision Command.
     *
     * @param driverController
     * @param visionSubsystem
     */
    public DefaultVisionCommand(OperatorInput driverController,
        VisionSubsystem visionSubsystem) {

        this.driverController = driverController;
        this.visionSubsystem  = visionSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(visionSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        visionSubsystem.setCameraMotorSpeed(driverController.getCameraMotorSpeed());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }
}