package frc.robot.commands.auto;

import static frc.robot.Constants.ArmConstants.GROUND_PICKUP_AUTO_POSITION;
import static frc.robot.Constants.ArmConstants.getScoringPosition;
import static frc.robot.Constants.VisionConstants.VisionTarget.APRILTAG_GRID;
import static frc.robot.commands.drive.DriveFastOnHeadingCommand.Direction.backward;
import static frc.robot.commands.drive.DriveFastOnHeadingCommand.Direction.forward;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants.AutoLane;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.Constants.VisionConstants.VisionTarget;
import frc.robot.commands.RunnymedeCommandBase;
import frc.robot.commands.arm.CompactCommand;
import frc.robot.commands.arm.ExtendArmCommand;
import frc.robot.commands.arm.MoveArmToPositionCommand;
import frc.robot.commands.arm.OpenPincherCommand;
import frc.robot.commands.arm.PickupGamePieceCommand;
import frc.robot.commands.arm.ReleaseCommand;
import frc.robot.commands.arm.ScoreAutoCommand;
import frc.robot.commands.drive.DriveFastOnHeadingCommand;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.commands.drive.DriveToFieldElementCommand;
import frc.robot.commands.drive.DriveToGamePieceCommand;
import frc.robot.commands.drive.RotateToHeadingCommand;
import frc.robot.commands.drive.RotateToHeadingCommand.DirectionOfRotation;
import frc.robot.commands.drive.SetGyroHeadingCommand;
import frc.robot.commands.vision.SetVisionTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DoubleDownAutoCommand extends SequentialCommandGroup {

    public DoubleDownAutoCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem,
        SendableChooser<AutoLane> startingLaneChooser) {

        final AutoLane startingLane = startingLaneChooser.getSelected();
        final Alliance alliance     = DriverStation.getAlliance();

        StringBuilder  sb           = new StringBuilder("Auto Selections: ");
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
        if (startingLane != AutoLane.BOTTOM && startingLane != AutoLane.TOP) {
            System.out.println("*** ERROR *** Invalid starting lane " + startingLane + ". Second piece not scored");
            return;
        }

        /*
         * Leave and lower the arm.
         * First back out to clear the grid
         * Then, drive out as fast as is safe,
         * while setting up the vision target
         * and arm for intake
         */
        RunnymedeCommandBase driveOutCmd;
        if (startingLane == AutoLane.BOTTOM) {
            // drive over the bump
            driveOutCmd = new DriveFastOnHeadingCommand(180, backward, 380, false, driveSubsystem);
        }
        else {
            // no bump
            driveOutCmd = new DriveFastOnHeadingCommand(180, backward, 360, false, driveSubsystem);
        }
        addCommands(driveOutCmd
            .alongWith(new ExtendArmCommand(0, armSubsystem)
                .andThen(new OpenPincherCommand(armSubsystem)
                    .andThen(new MoveArmToPositionCommand(GROUND_PICKUP_AUTO_POSITION, armSubsystem))))
            .alongWith(new SetVisionTargetCommand(VisionTarget.CUBE_GROUND, visionSubsystem)));

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
        addCommands(new DriveToGamePieceCommand(VisionTarget.CUBE_GROUND, .3, driveSubsystem, visionSubsystem, armSubsystem));
        // pickup cube needs you to drive through the piece or else it bounces away
        addCommands(new PickupGamePieceCommand(GamePiece.CUBE, null, armSubsystem)
        // .deadlineWith(new DriveForwardCommand(.1, 30, true, driveSubsystem))
        );

        /*
         * Step 3 - Score the cube
         *
         * Turn around to face the grid
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
        Constants.ArmPosition scorePosition = getScoringPosition(GamePiece.CUBE, ScoringRow.TOP);
        RunnymedeCommandBase  driveBackCmd;
        if (startingLane == AutoLane.BOTTOM) {
            // bump - drive safely
            driveBackCmd = new DriveFastOnHeadingCommand(180.0, forward, 230, false, driveSubsystem);
            // driveBackCmd = new DriveOnHeadingCommand(180.0, 0.6, 230, false, driveSubsystem);
        }
        else {
            // no bump - drive fast
            driveBackCmd = new DriveFastOnHeadingCommand(180.0, forward, 230, false, driveSubsystem);
        }

        addCommands(
            driveBackCmd
                .alongWith(new SetVisionTargetCommand(APRILTAG_GRID, visionSubsystem))
                .deadlineWith(new MoveArmToPositionCommand(scorePosition, armSubsystem)) // interruptible
        );

        /*
         * Track the April tag back to the scoring location.
         */
        addCommands(new SetVisionTargetCommand(APRILTAG_GRID, visionSubsystem));
        addCommands(new DriveToFieldElementCommand(APRILTAG_GRID, 0.35, driveSubsystem, visionSubsystem)
            .alongWith(new MoveArmToPositionCommand(scorePosition, armSubsystem)) // must finish
        );

        /*
         * Score the cube
         */
        addCommands(new WaitCommand(.1));
        addCommands(new ReleaseCommand(armSubsystem));

        /*
         * Back up to clear the grid then compact.
         */
        addCommands(new DriveOnHeadingCommand(180, -.2, 10, driveSubsystem));
        addCommands(new CompactCommand(armSubsystem));

    }
}

