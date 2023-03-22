package frc.robot.commands.vision;

import frc.robot.Constants;
import frc.robot.commands.RunnymedeCommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class SetVisionTargetCommand extends RunnymedeCommandBase {

    private final VisionSubsystem                        visionSubsystem;
    private final Constants.VisionConstants.VisionTarget target;

    public SetVisionTargetCommand(Constants.VisionConstants.VisionTarget target, VisionSubsystem visionSubsystem) {

        this.visionSubsystem = visionSubsystem;
        this.target          = target;

        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {

        logCommandStart("Vision target : " + target);

        if (!(target.getCameraView() == Constants.VisionConstants.CameraView.HIGH
            || target.getCameraView() == Constants.VisionConstants.CameraView.LOW)) {

            // Only HIGH or LOW are valid for this command, otherwise cancel
            log("Unsupported camera view: " + target.getCameraView() + ". Cancelling.");
            this.cancel();

            return;
        }

        visionSubsystem.setVisionTarget(target);
    }

    @Override
    public boolean isFinished() {

        if (visionSubsystem.getCurrentVisionTarget() == target && visionSubsystem.isCameraInPositionForTarget()) {
            setFinishReason("Vision system positioned");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        // Stop the motor
        visionSubsystem.setCameraMotorSpeed(0);

        logCommandEnd(interrupted);
    }
}
