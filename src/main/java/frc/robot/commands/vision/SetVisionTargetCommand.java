package frc.robot.commands.vision;

import frc.robot.Constants;
import frc.robot.commands.RunnymedeCommandBase;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.VisionConstants.CameraView.*;

public class SetVisionTargetCommand extends RunnymedeCommandBase {

    private final VisionSubsystem                        visionSubsystem;
    private final Constants.VisionConstants.VisionTarget target;
    private boolean targetValid = false;

    public SetVisionTargetCommand(Constants.VisionConstants.VisionTarget target, VisionSubsystem visionSubsystem) {

        this.visionSubsystem = visionSubsystem;
        this.target          = target;

        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {

        logCommandStart("Vision target : " + target);
        Constants.VisionConstants.CameraView tv = target.getCameraView();

        if (!(tv == HIGH || tv == MID || tv == LOW)) {

            // Only HIGH or LOW are valid for this command, otherwise cancel
            log("Unsupported camera view: " + target.getCameraView() + ". Cancelling.");
            this.cancel();
            targetValid = false;
            return;
        }

        targetValid = true;
        visionSubsystem.setVisionTarget(target);
    }

    @Override
    public boolean isFinished() {

        if (targetValid) {
            if (visionSubsystem.getCurrentVisionTarget() == target && visionSubsystem.isCameraInPositionForTarget()) {
                setFinishReason("Vision system positioned");
                return true;
            } else {
                return false;
            }
        } else {
            setFinishReason("Invalid target - finishing.");
            return true;
        }

    }

    @Override
    public void end(boolean interrupted) {

        // Stop the motor
        visionSubsystem.setCameraMotorSpeed(0);

        logCommandEnd(interrupted);
    }
}
