package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.VisionConstants.CameraView;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionTargetType;

public class SwitchVisionTargetCommand extends InstantCommand {

    /**
     * Switch the vision target to the specified {@link VisionTargetType}.
     *
     * @param visionTargetType requested vision target type
     * @param visionSubsystem the vision subsystem
     */
    public SwitchVisionTargetCommand(VisionTargetType visionTargetType, VisionSubsystem visionSubsystem) {

        super(() -> {

            VisionTargetType nextVisionTargetType = visionTargetType;

            // Start printing a message to the console
            System.out.print("SwitchVisionTargetCommand: ");

            if (nextVisionTargetType != null) {

                // If the vision target was passed in, then use that value
                System.out.println("new target " + nextVisionTargetType);
            }
            else {

                // If the vision target was not passed in, then set the vision target to the
                // next value in the enum
                VisionTargetType currentTargetType = visionSubsystem.getCurrentVisionTargetType();
                System.out.print("Current target type is " + currentTargetType);

                int idx     = currentTargetType.ordinal();
                int nextIdx = (idx + 1) % VisionTargetType.values().length;

                nextVisionTargetType = VisionTargetType.values()[nextIdx];

                System.out.println("Switch to next value " + nextVisionTargetType);
            }

            visionSubsystem.setVisionTargetType(nextVisionTargetType);

            /*
             * If in teleop, then automatically schedule the movement of the camera to the appropriate location
             * In auto, this will be done when required by the autocommand.
             */
            if (DriverStation.isTeleopEnabled()) {

                // Set the camera based on the vision target
                switch (nextVisionTargetType) {

                case CONE:
                case CUBE:
                    if (visionSubsystem.getCameraView() != CameraView.LOW) {
                        CommandScheduler.getInstance()
                            .schedule(new SetCameraViewCommand(CameraView.LOW, visionSubsystem));
                    }
                    break;

                case CONE_POST_LOW:
                case CONE_POST_HIGH:
                case TAG:
                    if (visionSubsystem.getCameraView() != CameraView.HIGH) {
                        CommandScheduler.getInstance()
                            .schedule(new SetCameraViewCommand(CameraView.HIGH, visionSubsystem));
                    }
                    break;

                default:
                    // Do nothing - do not move the camera
                    break;
                }
            }

        });
    }
}
