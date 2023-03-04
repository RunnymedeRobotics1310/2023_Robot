package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraView;
import frc.robot.subsystems.VisionSubsystem;

public class SetCameraViewCommand extends CommandBase {

    private final CameraView      newCameraView;
    private final VisionSubsystem visionSubsystem;

    // fixme: do everything - see table
    // https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#

    public SetCameraViewCommand(CameraView newCameraView, VisionSubsystem visionSubsystem) {

        this.newCameraView   = newCameraView;
        this.visionSubsystem = visionSubsystem;

        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {

        CameraView currentCameraView = visionSubsystem.getCameraView();

        System.out.println("SetCameraViewCommand started, current camera view : " + currentCameraView
            + ", target camera view " + newCameraView);

        if (!(newCameraView == CameraView.HIGH || newCameraView == CameraView.LOW)) {

            // Only HIGH or LOW are valid for this command, otherwise cancel
            visionSubsystem.setCameraMotorSpeed(0);
            this.cancel();

        }
        else if (visionSubsystem.getCameraView() == newCameraView) {

            // If already at the selected position, then cancel
            visionSubsystem.setCameraMotorSpeed(0);
            this.cancel();

        }
        else if (newCameraView == CameraView.HIGH) {

            // Run the motor forward to raise the camera view
            visionSubsystem.setCameraMotorSpeed(VisionConstants.MAX_CAMERA_MOTOR_SPEED);

        }
        else {

            // Run the motor in reverse to lower the camera view
            visionSubsystem.setCameraMotorSpeed(-VisionConstants.MAX_CAMERA_MOTOR_SPEED);

        }
    }

    @Override
    public void execute() {
        // Nothing to do here. Wait for the camera to get to the position
    }

    @Override
    public boolean isFinished() {

        if (visionSubsystem.getCameraView() == newCameraView) {
            return true;
        }

        return false;
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

        System.out.println("current camera view " + visionSubsystem.getCameraView());
    }
}
