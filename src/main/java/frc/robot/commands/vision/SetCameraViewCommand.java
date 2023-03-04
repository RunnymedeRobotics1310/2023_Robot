package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraView;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.VisionSubsystem;

public class SetCameraViewCommand extends CommandBase {

    private final CameraView      newCameraView;
    private final OperatorInput   operatorInput;
    private final VisionSubsystem visionSubsystem;

    // fixme: do everything - see table
    // https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#

    public SetCameraViewCommand(CameraView newCameraView, OperatorInput operatorInput,
        VisionSubsystem visionSubsystem) {

        this.newCameraView   = newCameraView;
        this.operatorInput   = operatorInput;
        this.visionSubsystem = visionSubsystem;

        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {

        CameraView currentCameraView = visionSubsystem.getCameraView();

        System.out.println("SetCameraViewCommand started, current camera view : " + currentCameraView
            + ", target camera view " + newCameraView);

        if (visionSubsystem.getCameraView() == newCameraView) {
            visionSubsystem.setCameraMotorSpeed(0);
            this.cancel();
        }
        else if (newCameraView == CameraView.HIGH) {
            visionSubsystem.setCameraMotorSpeed(VisionConstants.MAX_CAMERA_MOTOR_SPEED);
        }
        else {
            visionSubsystem.setCameraMotorSpeed(-VisionConstants.MAX_CAMERA_MOTOR_SPEED);
        }
    }

    @Override
    public boolean isFinished() {

        // Is it possible to cancel this command?
        // The camera will stop in an unknown position.
        if (operatorInput.isCancel()) {
            return true;
        }

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

        System.out.println("Current camera view " + visionSubsystem.getCameraView());
    }
}
