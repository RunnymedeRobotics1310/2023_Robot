package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.Constants.VisionConstants.VisionTarget;
import frc.robot.commands.arm.CompactCommand;
import frc.robot.commands.arm.PickupGamePieceCommand;
import frc.robot.commands.arm.ReleaseCommand;
import frc.robot.commands.arm.ScoreAutoCommand;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.commands.drive.DriveToTargetCommand;
import frc.robot.commands.drive.RotateToHeadingCommand;
import frc.robot.commands.drive.SetGyroHeadingCommand;
import frc.robot.commands.vision.SetVisionTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class MiddleMayhemAutoCommand extends SequentialCommandGroup {

    public MiddleMayhemAutoCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {

        /*
         * Set the to be facing the grid
         */
        addCommands(new SetGyroHeadingCommand(180, driveSubsystem));

        /*
         * Robot is holding a cone
         */
        addCommands(new InstantCommand(() -> {
            armSubsystem.setPincherEncoder(GamePiece.CONE.pincherEncoderCount);
        }));

        /*
         * Score the cone
         */
        addCommands(new ScoreAutoCommand(ScoringRow.TOP, armSubsystem));
        addCommands(new WaitCommand(.1));
        addCommands(new ReleaseCommand(armSubsystem));

        /*
         * Move to compact pose while backing up a bit to give some room to spin
         */
        addCommands(new CompactCommand(armSubsystem)
            .deadlineWith(new DriveOnHeadingCommand(180, -.2, 20, driveSubsystem)));

        /*
         * Rotate to face the field
         */
        addCommands(new RotateToHeadingCommand(0, driveSubsystem));

        /*
         * Step 2 - Exit the zone, and pick up a cube
         */

        /*
         * Drive over the charger and get the camera ready
         */
        addCommands(new DriveOnHeadingCommand(0, .65, 390, driveSubsystem)
            .deadlineWith(new SetVisionTargetCommand(VisionTarget.CUBE_GROUND, visionSubsystem)));

        /*
         * Pick up the cube
         */
        addCommands(new StartIntakeCommand(GamePiece.CUBE, armSubsystem, visionSubsystem)

            .deadlineWith(new WaitCommand(.5) // wait for the intake to get into position
                .andThen(new DriveToTargetCommand(VisionTarget.CUBE_GROUND, .3, driveSubsystem, visionSubsystem, armSubsystem)))

            .andThen(new PickupGamePieceCommand(GamePiece.CUBE, null, armSubsystem)));

        /*
         * Rotate back towards the grid with the game piece
         */
        addCommands(new RotateToHeadingCommand(180, driveSubsystem));

        /*
         * Drive up onto the charger (fast, then slower)
         */
        addCommands(new DriveOnHeadingCommand(180, .5, 190, false, driveSubsystem));
        addCommands(new DriveOnHeadingCommand(180, .3, 100, driveSubsystem));

        /*
         * Balance on the platform
         */
        addCommands(new WaitCommand(1));
        addCommands(new BalanceCommand(driveSubsystem));
    }
}

