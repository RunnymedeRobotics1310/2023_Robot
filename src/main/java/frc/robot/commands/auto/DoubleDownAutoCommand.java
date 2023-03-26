package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants.AutoLane;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.Constants.VisionConstants.VisionTarget;
import frc.robot.commands.arm.CompactCommand;
import frc.robot.commands.arm.PickupGamePieceCommand;
import frc.robot.commands.arm.ReleaseCommand;
import frc.robot.commands.arm.ScoreAutoCommand;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.commands.drive.*;
import frc.robot.commands.drive.RotateToHeadingCommand.DirectionOfRotation;
import frc.robot.commands.vision.SetVisionTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.commands.drive.DriveFastOnHeadingCommand.Direction.*;

public class DoubleDownAutoCommand extends SequentialCommandGroup {

    public DoubleDownAutoCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem,
        SendableChooser<AutoLane> startingLaneChooser) {

        final AutoLane startingLane = startingLaneChooser.getSelected();
        final Alliance alliance     = DriverStation.getAlliance();

        StringBuilder sb = new StringBuilder("Auto Selections: ");
        sb.append("Pattern: The Double Down ");
        sb.append("Starting Position:").append(startingLane).append(' ');
        sb.append("Alliance:").append(alliance);
        System.out.println(sb.toString());

        // If any of these are null, then there was some kind of error.
        if (startingLane == null) {
            System.out.println("*** ERROR *** null starting lane");
        }

        // Print an error if the alliance is not set
        if (alliance == null) {
            System.out.println("*** ERROR **** null Alliance ");
        }
        else if (alliance == Alliance.Invalid) {
            System.out.println("*** ERROR *** Invalid alliance");
        }

        /*
         * Set the gyro heading : for Double Down the robot is facing the grid.
         */
        addCommands(new SetGyroHeadingCommand(180, driveSubsystem));

        /*
         * The robot is holding a cone
         */
        addCommands(new InstantCommand(() -> {
            armSubsystem.setPincherEncoder(GamePiece.CONE.pincherEncoderCount);
        }));

        /*
         * Step 1 - Score the cone in the top row
         */
        addCommands(new ScoreAutoCommand(ScoringRow.TOP, GamePiece.CONE, armSubsystem));
        addCommands(new ReleaseCommand(armSubsystem));

        /*
         * Step 2: Exit the zone and pick up the cube
         *
         * Drive out of the zone and switch camera view
         * and set the intake to pick up a cube.
         */

        // Note: this command sets the intake to the correct position, but then cancels the
        // StartIntakeCommand when the DriveOnHeading command is complete.
        if (startingLane == AutoLane.BOTTOM) {
            // drive over the bump
            addCommands(new DriveOnHeadingCommand(180, -0.65, 340, driveSubsystem)
                .alongWith(new SetVisionTargetCommand(VisionTarget.CUBE_GROUND, visionSubsystem)
                .alongWith(new StartIntakeCommand(GamePiece.CUBE, armSubsystem, visionSubsystem))));
        } else if (startingLane == AutoLane.TOP) {
            // no bump
            addCommands(new DriveFastOnHeadingCommand(180, backward, 370, false, driveSubsystem)
                .alongWith(new SetVisionTargetCommand(VisionTarget.CUBE_GROUND, visionSubsystem)
                .alongWith(new StartIntakeCommand(GamePiece.CUBE, armSubsystem, visionSubsystem))));
        } else {
            System.out.println("*** ERROR *** Invalid starting lane " + startingLane + ". Second piece not scored");
            return;
        }

        /*
         * Rotate to face the cube
         *
         * Do not hit the wall with the intake *** DANGER **
         *
         * NOTE: The auto will at least score the cone and exit the zone if there was an error
         * in the auto chooser.
         */

        if (alliance != Alliance.Blue && alliance != Alliance.Red) {
            System.out.println("*** ERROR *** Unknown alliance " + alliance + ". Second piece not scored");
            return;
        }

        /*
         * Rotate away from the wall *** DANGER ***
         */
        if (alliance == Alliance.Red && startingLane == AutoLane.BOTTOM
            || alliance == Alliance.Blue && startingLane == AutoLane.TOP) {

            addCommands(new RotateToHeadingCommand(5, DirectionOfRotation.COUNTER_CLOCKWISE, driveSubsystem));
        }
        else {

            addCommands(new RotateToHeadingCommand(355, DirectionOfRotation.CLOCKWISE, driveSubsystem));
        }

        /*
         * Pick up the cube using vision
         *
         * NOTE: The intake is already in the correct position
         */
        addCommands(new DriveToTargetCommand(VisionTarget.CUBE_GROUND, .2, driveSubsystem,
            visionSubsystem, armSubsystem));
        addCommands(new PickupGamePieceCommand(GamePiece.CUBE, null, armSubsystem));

        /*
         * Step 3 - Score the cube
         *
         * Turn around to face the grid
         *
         * FIXME: Does the direction of rotation matter? Can we use the shortest path?
         */
        if (alliance == Alliance.Red && startingLane == AutoLane.BOTTOM
            || alliance == Alliance.Blue && startingLane == AutoLane.TOP) {

            addCommands(new RotateToHeadingCommand(180, DirectionOfRotation.COUNTER_CLOCKWISE, driveSubsystem));

        }
        else {

            addCommands(new RotateToHeadingCommand(180, DirectionOfRotation.CLOCKWISE, driveSubsystem));
        }

        /*
         * Drive back towards the grid until past the charging station
         * Set the Vision target so that it is ready
         */
        if (startingLane == AutoLane.BOTTOM) {
            // bump - drive safely
            addCommands(new DriveOnHeadingCommand(180.0, 0.6, 250, false, driveSubsystem)
                .alongWith(new ScoreAutoCommand(ScoringRow.TOP, GamePiece.CUBE, armSubsystem))
                .alongWith(new SetVisionTargetCommand(VisionTarget.APRILTAG_GRID, visionSubsystem)));
        } else {
            // no bump - drive fast
            addCommands(new DriveFastOnHeadingCommand(180.0, forward, 250, false,  driveSubsystem)
                .alongWith(new ScoreAutoCommand(ScoringRow.TOP, GamePiece.CUBE, armSubsystem))
                .alongWith(new SetVisionTargetCommand(VisionTarget.APRILTAG_GRID, visionSubsystem)));
        }

        /*
         * Track the April tag back to the scoring location.
         *
         * FIXME: should we use .alongWith instead of .deadlineWith to make sure both commands are
         * finished before dropping?
         */
        addCommands(new DriveToTargetCommand(VisionTarget.APRILTAG_GRID, 0.35, driveSubsystem, visionSubsystem, armSubsystem));

        /*
         * Score the cube
         */
        addCommands(new ReleaseCommand(armSubsystem));

        /*
         * Back up while compacting to not hit the mid cube scoring spot.
         */

        addCommands(new CompactCommand(armSubsystem)
            .deadlineWith(new DriveOnHeadingCommand(180, -.2, 10, driveSubsystem)));

    }
}

