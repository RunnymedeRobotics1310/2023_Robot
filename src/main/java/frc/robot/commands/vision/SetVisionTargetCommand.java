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

        visionSubsystem.setVisionTarget(target);

        if (!(target.getCameraView() == Constants.VisionConstants.CameraView.HIGH
            || target.getCameraView() == Constants.VisionConstants.CameraView.LOW)) {

            // Only HIGH or LOW are valid for this command, otherwise cancel
            visionSubsystem.setCameraMotorSpeed(0);
            System.out.println("ConfigureCameraCommand: unsupported camera view: " + target.getCameraView() + ". Cancelling.");
            this.cancel();

            return;
        }

    }

    @Override
    public void execute() {

        Constants.VisionConstants.CameraView currentCameraView = visionSubsystem.getCameraView();

        if (currentCameraView == target.getCameraView()) {

            // Camera is already in position.
            visionSubsystem.setCameraMotorSpeed(0);

        }
        else if (target.getCameraView() == Constants.VisionConstants.CameraView.HIGH) {

            // Run the motor forward to raise the camera view
            visionSubsystem.setCameraMotorSpeed(Constants.VisionConstants.MAX_CAMERA_MOTOR_SPEED);

        }
        else {

            // Run the motor in reverse to lower the camera view
            visionSubsystem.setCameraMotorSpeed(-Constants.VisionConstants.MAX_CAMERA_MOTOR_SPEED);

        }
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
