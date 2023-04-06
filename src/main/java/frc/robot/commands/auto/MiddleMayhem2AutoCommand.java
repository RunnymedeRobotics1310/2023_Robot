package frc.robot.commands.auto;

import static frc.robot.commands.drive.DriveFastOnHeadingCommand.Direction.backward;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.Constants.VisionConstants.VisionTarget;
import frc.robot.commands.arm.ExtendArmCommand;
import frc.robot.commands.arm.MoveArmToPositionCommand;
import frc.robot.commands.arm.OpenPincherCommand;
import frc.robot.commands.arm.PickupGamePieceCommand;
import frc.robot.commands.arm.ReleaseCommand;
import frc.robot.commands.arm.ScoreAutoCommand;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.commands.drive.DriveFastOnHeadingCommand;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.commands.drive.DriveToGamePieceCommand;
import frc.robot.commands.drive.ResetGyroPitchCommand;
import frc.robot.commands.drive.RotateToHeadingCommand;
import frc.robot.commands.drive.SetGyroHeadingCommand;
import frc.robot.commands.vision.SetVisionTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class MiddleMayhem2AutoCommand extends SequentialCommandGroup {

    public MiddleMayhem2AutoCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {

        StringBuilder sb = new StringBuilder("Auto Selections: ");
        sb.append("Pattern: Middle Mayhem ");
        sb.append("Alliance: ").append(DriverStation.getAlliance());
        System.out.println(sb.toString());

        /*
         * Set the to be facing the grid
         */
        addCommands(new SetGyroHeadingCommand(180, driveSubsystem));
        addCommands(new ResetGyroPitchCommand(driveSubsystem));

        /*
         * Robot is holding a cone
         */
        addCommands(new InstantCommand(() -> {
            armSubsystem.setPincherEncoder(GamePiece.CONE.pincherEncoderCount);
        }));

        /*
         * Score the cone
         */
        addCommands(new ScoreAutoCommand(ScoringRow.TOP, GamePiece.CONE, armSubsystem));
        addCommands(new ReleaseCommand(armSubsystem));

        /*
         * Move to compact pose while backing up a bit to give some room to spin
         */
        // mount charger


        addCommands(new DriveFastOnHeadingCommand(180, backward, 200, false, driveSubsystem)
            .andThen(new DriveOnHeadingCommand(180, -.3, 70, false, driveSubsystem)
                .andThen(new DriveOnHeadingCommand(180, -.5, 40, false, driveSubsystem)))
            .alongWith(new ExtendArmCommand(0, armSubsystem)
                .andThen(new MoveArmToPositionCommand(Constants.ArmConstants.GROUND_PICKUP_AUTO_POSITION, armSubsystem)
                    .andThen(new OpenPincherCommand(armSubsystem))))
            .alongWith(new SetVisionTargetCommand(VisionTarget.CUBE_GROUND, visionSubsystem)));

        // rotate to cube
        addCommands(new RotateToHeadingCommand(0, driveSubsystem));

        /*
         * We are now over the charge station. Finish the drive.
         * Now that we're clear, it's safe to get the arm into the final intake position
         */
        addCommands(new DriveOnHeadingCommand(0, .65, 30, driveSubsystem));

        /*
         * Pick up the cube
         */
        addCommands(new DriveToGamePieceCommand(VisionTarget.CUBE_GROUND, .3, driveSubsystem, visionSubsystem, armSubsystem)
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

