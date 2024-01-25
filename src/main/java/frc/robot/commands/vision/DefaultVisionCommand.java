package frc.robot.commands.vision;

import frc.robot.commands.RunnymedeCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.VisionSubsystem;

public class DefaultVisionCommand extends RunnymedeCommand {

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


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        visionSubsystem.setCameraMotorSpeed(driverController.getCameraMotorSpeed());
    }

}