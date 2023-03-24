package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.AutoLane;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.Constants.VisionConstants.VisionTarget;
import frc.robot.commands.arm.CompactCommand;
import frc.robot.commands.arm.PickupGamePieceCommand;
import frc.robot.commands.arm.ReleaseCommand;
import frc.robot.commands.arm.ScoreAutoCommand;
import frc.robot.commands.arm.ScoreCommand;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.commands.drive.DriveToTargetCommand;
import frc.robot.commands.drive.RotateToHeadingCommand;
import frc.robot.commands.drive.RotateToHeadingCommand.DirectionOfRotation;
import frc.robot.commands.drive.SetGyroHeadingCommand;
import frc.robot.commands.vision.SetVisionTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DoubleDownAutoCommand extends SequentialCommandGroup {

    private AutoLane startingLane = null;
    private Alliance alliance     = null;

    public DoubleDownAutoCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem,
        SendableChooser<AutoLane> startingLaneChooser) {

        startingLane = startingLaneChooser.getSelected();
        alliance     = DriverStation.getAlliance();

        StringBuilder sb = new StringBuilder();
        sb.append("Auto Selections");
        sb.append("\n   Starting Position :").append(startingLane);
        sb.append("\nAlliance             :").append(alliance);

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
        addCommands(new ScoreAutoCommand(ScoringRow.TOP, armSubsystem));

        addCommands(new WaitCommand(.1));
        addCommands(new ReleaseCommand(armSubsystem));

        /*
         * Step 2: Exit the zone and pick up the cube
         *
         * Drive out of the zone and switch camera view
         * and set the intake to pick up a cube.
         */

        double exitZoneDistance = 330;
        if (startingLane == AutoLane.BOTTOM) {
            exitZoneDistance = 340;
        }

        // Note: this command sets the intake to the correct position, but then cancels the
        // StartIntakeCommand when the DriveOnHeading command is complete.

        addCommands(new DriveOnHeadingCommand(0, 0.65, exitZoneDistance, driveSubsystem)
            .deadlineWith(new StartIntakeCommand(GamePiece.CUBE, armSubsystem, visionSubsystem))
            .deadlineWith(new SetVisionTargetCommand(VisionTarget.CUBE_GROUND, visionSubsystem)));

        /*
         * Rotate to face the cube
         *
         * Do not hit the wall with the intake *** DANGER **
         *
         * NOTE: The auto will at least score the cone and exit the zone if there was an error
         * in the auto chooser.
         */

        if (startingLane != AutoLane.TOP && startingLane != AutoLane.BOTTOM) {
            System.out.println("*** ERROR *** Invalid starting lane " + startingLane + ". Second piece not scored");
            return;
        }

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
        addCommands(new DriveToTargetCommand(VisionTarget.CUBE_GROUND, .3, driveSubsystem,
            visionSubsystem, armSubsystem)
            .andThen(new PickupGamePieceCommand(GamePiece.CUBE, null, armSubsystem)));

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
        addCommands(new DriveOnHeadingCommand(180.0, 0.6, 250, driveSubsystem)
            .deadlineWith(new SetVisionTargetCommand(VisionTarget.APRILTAG_GRID, visionSubsystem)));

        /*
         * Track the April tag back to the scoring location.
         *
         * FIXME: should we use .alongWith instead of .deadlineWith to make sure both commands are
         * finished before dropping?
         */
        addCommands(new DriveToTargetCommand(VisionTarget.APRILTAG_GRID, 0.35, driveSubsystem, visionSubsystem, armSubsystem)
            .deadlineWith(new ScoreCommand(ScoringRow.TOP, armSubsystem)));

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

