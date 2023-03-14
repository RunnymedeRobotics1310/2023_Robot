package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem;

public class SetVisionTargetCommand extends CommandBase {

    private final VisionSubsystem                        visionSubsystem;
    private final Constants.VisionConstants.VisionTarget target;

    public SetVisionTargetCommand(Constants.VisionConstants.VisionTarget target, VisionSubsystem visionSubsystem) {

        this.visionSubsystem = visionSubsystem;
        this.target          = target;

        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {

        System.out.println("SetVisionTargetCommand started, target vision target : " + target);

        if (!(target.getCameraView() == Constants.VisionConstants.CameraView.HIGH
            || target.getCameraView() == Constants.VisionConstants.CameraView.LOW)) {

            // Only HIGH or LOW are valid for this command, otherwise cancel
            System.out.println("ConfigureCameraCommand: unsupported camera view: " + target.getCameraView() + ". Cancelling.");
            this.cancel();

            return;
        }

        visionSubsystem.setVisionTarget(target);
    }

    @Override
    public boolean isFinished() {

        return visionSubsystem.getCurrentVisionTarget() == target && visionSubsystem.isCameraInPositionForTarget();
    }

    @Override
    public void end(boolean interrupted) {

        // Stop the motor
        visionSubsystem.setCameraMotorSpeed(0);

        // Print a debug message
        if (interrupted) {
            System.out.print("SetCameraViewCommand interrupted: ");
        }
        else {
            System.out.print("SetCameraViewCommand ended: ");
        }

        System.out.println("Current vision target " + visionSubsystem.getCurrentVisionTarget() + " current view: "
            + visionSubsystem.getCameraView());
    }
}
