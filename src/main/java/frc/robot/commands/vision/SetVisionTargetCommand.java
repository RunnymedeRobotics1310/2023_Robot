package frc.robot.commands.vision;

import static frc.robot.Constants.VisionConstants.CameraView.HIGH;
import static frc.robot.Constants.VisionConstants.CameraView.LOW;
import static frc.robot.Constants.VisionConstants.CameraView.MID;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants.CameraView;
import frc.robot.commands.RunnymedeCommand;
import frc.robot.subsystems.VisionSubsystem;

public class SetVisionTargetCommand extends RunnymedeCommand {

    private final VisionSubsystem                        visionSubsystem;
    private final Constants.VisionConstants.VisionTarget target;
    private boolean                                      targetValid = false;

    public SetVisionTargetCommand(Constants.VisionConstants.VisionTarget target, VisionSubsystem visionSubsystem) {

        this.visionSubsystem = visionSubsystem;
        this.target          = target;

        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {

        logCommandStart("Vision target : " + target);
        CameraView targetCameraView = target.getCameraView();

        if (!(targetCameraView == HIGH || targetCameraView == MID || targetCameraView == LOW)) {

            // Only HIGH, LOW and MID are valid for this command, otherwise cancel
            log("Unsupported camera view: " + target.getCameraView() + ". Cancelling.");
            targetValid = false;
            return;
        }

        targetValid = true;
        visionSubsystem.setVisionTarget(target);
    }

    @Override
    public void execute() {

        if (targetValid) {

            if (!(visionSubsystem.getCurrentVisionTarget() == target
                && visionSubsystem.isCameraInPositionForTarget())) {

                visionSubsystem.setVisionTarget(target);
            }
        }
    }

    @Override
    public boolean isFinished() {

        if (targetValid) {
            if (visionSubsystem.getCurrentVisionTarget() == target && visionSubsystem.isCameraInPositionForTarget()) {
                setFinishReason("Vision system positioned");
                return true;
            }
            else {
                return false;
            }
        }
        else {
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
