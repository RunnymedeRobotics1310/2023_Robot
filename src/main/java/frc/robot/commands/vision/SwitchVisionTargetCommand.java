package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionTargetType;

public class SwitchVisionTargetCommand extends InstantCommand {

    /**
     * Switches the vision target to the next {@link VisionTargetType} enum value
     *
     * @param visionSubsystem
     */
    public SwitchVisionTargetCommand(VisionSubsystem visionSubsystem) {

        this(null, visionSubsystem);
    }

    /**
     * Switch the vision target to the specified {@link VisionTargetType}.
     *
     * @param visionTargetType requested vision target type
     * @param visionSubsystem
     */
    public SwitchVisionTargetCommand(VisionTargetType visionTargetType, VisionSubsystem visionSubsystem) {

        super(() -> {

            // Print a message to the console

            System.out.print("SwitchVisionTargetCommand");
            if (visionTargetType != null) {
                System.out.println(": Switch to vision target " + visionTargetType);
            }
            else {
                System.out.println(": Switch to next value");
            }

            // If the vision target was passed in, then use that value

            if (visionTargetType != null) {
                visionSubsystem.setVisionTargetType(visionTargetType);
                return;
            }

            // If the vision target was not passed in, then set the vision target to the
            // next value in the enum

            VisionTargetType currentTargetType = visionSubsystem.getCurrentVisionTargetType();
            System.out.println("Current target type is " + currentTargetType);

            int idx = currentTargetType.ordinal();
            int nextIdx = (idx + 1) % VisionTargetType.values().length;

            VisionTargetType nextVisionTargetType = VisionTargetType.values()[nextIdx];
            System.out.println("Setting vision target type to " + nextVisionTargetType);

            visionSubsystem.setVisionTargetType(nextVisionTargetType);
        });
    }
}
