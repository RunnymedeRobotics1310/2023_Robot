package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.AutoAction;
import frc.robot.Constants.AutoConstants.AutoLane;
import frc.robot.Constants.AutoConstants.Orientation;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.Constants.GameConstants.Zone;
import frc.robot.Constants.VisionConstants.VisionTarget;
import frc.robot.commands.arm.CompactCommand;
import frc.robot.commands.arm.PickupGamePieceCommand;
import frc.robot.commands.arm.ReleaseCommand;
import frc.robot.commands.arm.ScoreAutoCommand;
import frc.robot.commands.arm.ScoreCommand;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.commands.drive.DriveToTargetCommand;
import frc.robot.commands.drive.RotateToHeadingCommand;
import frc.robot.commands.drive.RotateToHeadingCommand.DirectionOfRotation;
import frc.robot.commands.drive.SetGyroHeadingCommand;
import frc.robot.commands.vision.SetVisionTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {

    private AutoLane              startingLane           = null;
    private GamePiece             currentGamePiece       = null;
    private Orientation           currentOrientation     = null;
    private Zone                  currentZone            = null;

    private Alliance              alliance               = null;

    private AutoAction            firstGamePieceScoring  = null;
    private AutoAction            exitZoneAction         = null;
    private AutoAction            secondGamePieceScoring = null;
    private AutoAction            balanceAction          = null;

    private final ArmSubsystem    armSubsystem;
    private final DriveSubsystem  driveSubsystem;
    private final VisionSubsystem visionSubsystem;


    public AutonomousCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem,
        SendableChooser<AutoLane> startingLaneChooser,
        SendableChooser<GamePiece> loadedGamePieceChooser,
        SendableChooser<Orientation> startingOrientationChooser,
        SendableChooser<AutoAction> firstGamePieceScoringChooser,
        SendableChooser<AutoAction> exitZoneActionChooser,
        SendableChooser<AutoAction> secondGamePieceScoringChooser,
        SendableChooser<AutoAction> balanceChooser) {

        this.driveSubsystem  = driveSubsystem;
        this.armSubsystem    = armSubsystem;
        this.visionSubsystem = visionSubsystem;

        // Default is to do nothing.
        // If more commands are added, the instant command will end and
        // the next command will be executed.
        addCommands(new InstantCommand());

        startingLane           = startingLaneChooser.getSelected();
        currentGamePiece       = loadedGamePieceChooser.getSelected();
        currentOrientation     = startingOrientationChooser.getSelected();

        firstGamePieceScoring  = firstGamePieceScoringChooser.getSelected();
        exitZoneAction         = exitZoneActionChooser.getSelected();
        secondGamePieceScoring = secondGamePieceScoringChooser.getSelected();
        balanceAction          = balanceChooser.getSelected();

        alliance               = DriverStation.getAlliance();

        StringBuilder sb = new StringBuilder();
        sb.append("Auto Selections");
        sb.append("\n   Starting Position          :").append(startingLane);
        sb.append("\n   Starting Orientation      :").append(currentOrientation);
        sb.append("\n   Loaded Game Piece         :").append(currentGamePiece);
        sb.append("\n   First Game Piece Scoring  :").append(firstGamePieceScoring);
        sb.append("\n   Exit Zone Action          :").append(exitZoneAction);
        sb.append("\n   Second Game Piece Scoring :").append(secondGamePieceScoring);
        sb.append("\n   Balance Action            :").append(balanceAction);
        sb.append("\nAlliance                     :").append(alliance);

        System.out.println(sb.toString());

        // If any of these are null, then there was some kind of error.
        // FIXME: Is there anything we can do here?
        if (startingLane == null
            || currentOrientation == null
            || currentGamePiece == null
            || firstGamePieceScoring == null
            || exitZoneAction == null
            || secondGamePieceScoring == null
            || balanceAction == null) {

            System.out.println("*** ERROR - null found in auto pattern builder ***");
            return;
        }

        // Print an error if the alliance is not set
        if (alliance == null) {
            System.out.println("*** ERROR **** null Alliance ");
            return;
        }
        else if (alliance == Alliance.Invalid) {
            System.out.println("*** ERROR *** Invalid alliance");
            return;
        }

        // The robot always starts next to the grid
        currentZone = Zone.COMMUNITY;

        /*
         * Set the gyro heading if required
         */
        if (currentOrientation == Orientation.FACE_GRID) {
            addCommands(new SetGyroHeadingCommand(180, driveSubsystem));
        }
        else {
            addCommands(new SetGyroHeadingCommand(0, driveSubsystem));
        }

        /*
         * Compose the required auto commands for each of the steps in the auto
         * based on the selector values
         */
        addStep1Commands_ScoreFirstGamePiece();
        addStep2Commands_ExitZone();
        addStep3Commands_ScoreSecondGamePiece();
        addStep4Commands_Balance();
    }

    /**
     * Step 1 - Score first game piece
     */
    private void addStep1Commands_ScoreFirstGamePiece() {

        // The selector must be set to score low or score mid, otherwise
        // there is nothing to do

        if (!(firstGamePieceScoring == AutoAction.SCORE_BOTTOM
            || firstGamePieceScoring == AutoAction.SCORE_MIDDLE
            || firstGamePieceScoring == AutoAction.SCORE_TOP)) {
            return;
        }



        if (currentOrientation == Orientation.FACE_GRID) {

            switch (currentGamePiece) {
            case CUBE:
                // Robot is holding a cube in the gripper, set pincher to CUBE distance
                addCommands(new InstantCommand(() -> {
                    armSubsystem.setPincherEncoder(GamePiece.CUBE.pincherEncoderCount);
                }));
                break;
            case CONE:
                addCommands(new InstantCommand(() -> {
                    armSubsystem.setPincherEncoder(GamePiece.CONE.pincherEncoderCount);
                }));
                break;

            default:
                break;
            }

            ScoringRow scoringRow = null;
            switch (firstGamePieceScoring) {
            case SCORE_BOTTOM:
                scoringRow = ScoringRow.BOTTOM;
                break;
            case SCORE_MIDDLE:
                scoringRow = ScoringRow.MIDDLE;
                break;
            case SCORE_TOP:
                scoringRow = ScoringRow.TOP;
                break;

            }
            addCommands(new ScoreAutoCommand(scoringRow, armSubsystem));
            addCommands(new WaitCommand(.1));
            addCommands(new ReleaseCommand(armSubsystem));
        }
        else {
            // Currently facing the field

            // When facing the field, only a low delivery is allowed because a piece would be
            // balanced on the bumper
            if (firstGamePieceScoring == AutoAction.SCORE_MIDDLE
                || firstGamePieceScoring == AutoAction.SCORE_TOP) {
                System.out.println("Cannot score mid unless facing grid, overriding to score low");
            }

            // Drive Backward
            // NOTE: deposit the piece placed on the bumper in the low scoring position using
            // gravity and inertia (or maybe a piston?)

            // reverse and deposit the starting piece

            double speed = -0.3;
            addCommands(new DriveOnHeadingCommand(0, speed, 20, driveSubsystem));
        }

        // Now that the game piece is scored, we do not have a game piece
        currentGamePiece = GamePiece.NONE;

    }

    /**
     * Step 2 - Exit the zone, and optionally pick up a second game piece
     */
    private void addStep2Commands_ExitZone() {

        // An exit zone action must be selected, otherwise do nothing

        if (!(exitZoneAction == AutoAction.EXIT_ZONE
            || exitZoneAction == AutoAction.PICK_UP_CUBE
            || exitZoneAction == AutoAction.PICK_UP_CONE)) {
            return;
        }



        double exitZoneDistance = 330;
        if (startingLane == AutoLane.BOTTOM) {
            exitZoneDistance = 340;
        }
        else if (startingLane == AutoLane.MIDDLE) {
            exitZoneDistance = 390;
        }
        // Drive out of the zone and switch camera view
        // This command may cause a rotation to heading 0.
        if (exitZoneAction == AutoAction.PICK_UP_CUBE
            && startingLane != AutoLane.MIDDLE) {
            if (currentOrientation == Orientation.FACE_FIELD) {
                addCommands(new DriveOnHeadingCommand(0, 0.65, exitZoneDistance, driveSubsystem)
                    .deadlineWith(new StartIntakeCommand(GamePiece.CUBE, armSubsystem, visionSubsystem))
                    .deadlineWith(new SetVisionTargetCommand(VisionTarget.CUBE_GROUND, visionSubsystem)));
            }
            else {
                addCommands(new DriveOnHeadingCommand(180, -0.65, exitZoneDistance, driveSubsystem)
                    .deadlineWith(new StartIntakeCommand(GamePiece.CUBE, armSubsystem, visionSubsystem))
                    .deadlineWith(new SetVisionTargetCommand(VisionTarget.CUBE_GROUND, visionSubsystem)));
            }
        }
        else if (exitZoneAction == AutoAction.PICK_UP_CONE
            && startingLane != AutoLane.MIDDLE) {
            if (currentOrientation == Orientation.FACE_FIELD) {
                addCommands(new DriveOnHeadingCommand(0, 0.65, exitZoneDistance, driveSubsystem)
                    .deadlineWith(new StartIntakeCommand(GamePiece.CONE, armSubsystem, visionSubsystem))
                    .deadlineWith(new SetVisionTargetCommand(VisionTarget.CONE_GROUND, visionSubsystem)));
            }
            else {
                addCommands(new DriveOnHeadingCommand(180, -0.65, exitZoneDistance, driveSubsystem)
                    .deadlineWith(new StartIntakeCommand(GamePiece.CONE, armSubsystem, visionSubsystem))
                    .deadlineWith(new SetVisionTargetCommand(VisionTarget.CUBE_GROUND, visionSubsystem)));
            }
        }

        // Middle routine
        else if (exitZoneAction == AutoAction.PICK_UP_CUBE) {
            addCommands(new DriveOnHeadingCommand(180, -.2, 20, driveSubsystem));

            addCommands(new CompactCommand(armSubsystem)
                .deadlineWith(new RotateToHeadingCommand(0, null, driveSubsystem)));
            currentOrientation = Orientation.FACE_FIELD;
            addCommands(new DriveOnHeadingCommand(0, .65, exitZoneDistance, driveSubsystem)
                .deadlineWith(new SetVisionTargetCommand(VisionTarget.CUBE_GROUND, visionSubsystem)));

            addCommands(new StartIntakeCommand(GamePiece.CUBE, armSubsystem, visionSubsystem)
                .deadlineWith(new WaitCommand(.5)
                    .andThen(
                        new DriveToTargetCommand(VisionTarget.CUBE_GROUND, .3, driveSubsystem, visionSubsystem, armSubsystem)))
                .andThen(new PickupGamePieceCommand(GamePiece.CUBE, null, armSubsystem)));
            currentGamePiece = GamePiece.CUBE;
        }

        else if (exitZoneAction == AutoAction.PICK_UP_CONE) {
            addCommands(new DriveOnHeadingCommand(180, -.2, 20, driveSubsystem));

            addCommands(new CompactCommand(armSubsystem)
                .deadlineWith(new RotateToHeadingCommand(0, null, driveSubsystem)));
            currentOrientation = Orientation.FACE_FIELD;
            addCommands(new DriveOnHeadingCommand(0, .65, exitZoneDistance, driveSubsystem)
                .deadlineWith(new SetVisionTargetCommand(VisionTarget.CONE_GROUND, visionSubsystem)));

            addCommands(new StartIntakeCommand(GamePiece.CUBE, armSubsystem, visionSubsystem)
                .deadlineWith(new WaitCommand(.5)
                    .andThen(
                        new DriveToTargetCommand(VisionTarget.CUBE_GROUND, .3, driveSubsystem, visionSubsystem, armSubsystem)))
                .andThen(new PickupGamePieceCommand(GamePiece.CUBE, null, armSubsystem)));
            currentGamePiece = GamePiece.CONE;

        }

        else {
            addCommands(new DriveOnHeadingCommand(180, -0.65, exitZoneDistance, driveSubsystem)
                .andThen(new CompactCommand(armSubsystem)));
        }


        // Rotate to heading 0
        if (currentOrientation == Orientation.FACE_GRID
            && (exitZoneAction == AutoAction.PICK_UP_CUBE
                || exitZoneAction == AutoAction.PICK_UP_CONE)) {
            addCommands(new RotateToHeadingCommand(5, DirectionOfRotation.COUNTER_CLOCKWISE, driveSubsystem));
            currentOrientation = Orientation.FACE_FIELD;
        }



        currentZone = Zone.FIELD;

        /*
         * If a piece is not required, this portion is complete
         */
        if (exitZoneAction == AutoAction.PICK_UP_CUBE) {

            // Start the pickup, and at the same time, drive toward the vision target. This command
            // will end when the PickupGroundCommand ends, canceling the DriveToVisionTarget if
            // required.

            // addCommands(new DriveToTargetCommand(VisionTarget.CUBE_GROUND, .3, driveSubsystem, visionSubsystem, armSubsystem)
            // .andThen(new PickupGamePieceCommand(GamePiece.CUBE, null, armSubsystem)));

        }
        if (exitZoneAction == AutoAction.PICK_UP_CONE) {

            // addCommands(new DriveToTargetCommand(VisionTarget.CONE_GROUND, .3, driveSubsystem, visionSubsystem, armSubsystem)
            // .andThen(new PickupGamePieceCommand(GamePiece.CUBE, null, armSubsystem)));
        }
    }

    /**
     * Step 3 - Deliver a second game piece if required
     */
    private void addStep3Commands_ScoreSecondGamePiece() {

        // If a scoring location is not set, there is nothing to do.

        if (!(secondGamePieceScoring == AutoAction.SCORE_BOTTOM
            || secondGamePieceScoring == AutoAction.SCORE_MIDDLE
            || secondGamePieceScoring == AutoAction.SCORE_TOP)) {
            return;
        }

        // Check that grabbing a piece was scheduled

        if (!(exitZoneAction == AutoAction.PICK_UP_CONE
            || exitZoneAction == AutoAction.PICK_UP_CUBE)) {
            System.out.println("*** ERROR *** Cannot deliver a second piece if it was not picked up");
            // Do nothing
            return;
        }

        // Turn around, go back to the grid, and score
        addCommands(new RotateToHeadingCommand(180, DirectionOfRotation.COUNTER_CLOCKWISE, driveSubsystem));
        addCommands(new DriveOnHeadingCommand(180.0, 0.6, 250, driveSubsystem)
            .deadlineWith(new SetVisionTargetCommand(VisionTarget.APRILTAG_GRID, visionSubsystem)));
        addCommands(new DriveToTargetCommand(VisionTarget.APRILTAG_GRID, 0.35, driveSubsystem, visionSubsystem, armSubsystem)
            .deadlineWith(new ScoreCommand(ScoringRow.TOP, armSubsystem)));

        // Drop piece & compact
        addCommands(new ReleaseCommand(armSubsystem));
        addCommands(new DriveOnHeadingCommand(180, -.2, 10, driveSubsystem));
        addCommands(new CompactCommand(armSubsystem));

        currentOrientation = Orientation.FACE_GRID;
        currentZone        = Zone.COMMUNITY;
    }

    /**
     * Step 4 - Balance if required
     */
    private void addStep4Commands_Balance() {

        // If the balance command was not selected, there is nothing to do
        if (balanceAction != AutoAction.BALANCE) {
            return;
        }

        // Drive to the platform from community facing grid
        if (currentZone == Zone.COMMUNITY && currentOrientation == Orientation.FACE_GRID) {


            if ((alliance == Alliance.Red && startingLane == AutoLane.BOTTOM)
                || (alliance == Alliance.Blue && startingLane == AutoLane.TOP)) {

                System.out.println("Balance Red/Bot or Blue/Top");
                addCommands(new DriveOnHeadingCommand(180, -.3, 20, driveSubsystem));
                addCommands(new DriveOnHeadingCommand(270, -.3, 100, driveSubsystem));
                addCommands(new DriveOnHeadingCommand(180, -.5, 160, driveSubsystem));
            }
            else if ((alliance == Alliance.Red && startingLane == AutoLane.TOP)
                || (alliance == Alliance.Blue && startingLane == AutoLane.BOTTOM)) {

                System.out.println("Balance Red/Top or Blue/Bottom");
                addCommands(new DriveOnHeadingCommand(180, -.3, 20, driveSubsystem));
                addCommands(new DriveOnHeadingCommand(90, -.3, 100, driveSubsystem));
                addCommands(new DriveOnHeadingCommand(180, -.5, 160, driveSubsystem));
            }
            else {
                System.out.println("Balance Mid");
                addCommands(new DriveOnHeadingCommand(180, -.3, 160, driveSubsystem));
            }
        }
        else if (currentZone == Zone.FIELD && currentOrientation == Orientation.FACE_FIELD) { // Drive to the platform from field
                                                                                              // facing field


            if ((alliance == Alliance.Red && startingLane == AutoLane.BOTTOM)
                || (alliance == Alliance.Blue && startingLane == AutoLane.TOP)) {

                System.out.println("Balance Red/Bot or Blue/Top");
                addCommands(new DriveOnHeadingCommand(90, .3, 180, driveSubsystem));
                addCommands(new DriveOnHeadingCommand(0, -.3, 170, driveSubsystem));
            }
            else if ((alliance == Alliance.Red && startingLane == AutoLane.TOP)
                || (alliance == Alliance.Blue && startingLane == AutoLane.BOTTOM)) {

                System.out.println("Balance Red/Top or Blue/Bottom");
                addCommands(new DriveOnHeadingCommand(270, .3, 180, driveSubsystem));
                addCommands(new DriveOnHeadingCommand(0, -.3, 170, driveSubsystem));
            }
            else {
                System.out.println("Balance Mid");

                if (currentGamePiece != GamePiece.NONE) {
                    addCommands(new RotateToHeadingCommand(180, null, driveSubsystem));
                    currentOrientation = Orientation.FACE_GRID;
                    addCommands(new DriveOnHeadingCommand(180, .5, 190, false, driveSubsystem));
                    addCommands(new DriveOnHeadingCommand(180, .3, 100, driveSubsystem));
                }
                else {
                    addCommands(new DriveOnHeadingCommand(0, -.3, 170, driveSubsystem));
                }
            }
        }
        else if (currentZone == Zone.FIELD && currentOrientation == Orientation.FACE_GRID) { // from field facing grid


            if ((alliance == Alliance.Red && startingLane == AutoLane.BOTTOM)
                || (alliance == Alliance.Blue && startingLane == AutoLane.TOP)) {

                System.out.println("Balance Red/Bot or Blue/Top");
                addCommands(new DriveOnHeadingCommand(135, .1, 1, driveSubsystem));
                addCommands(new DriveOnHeadingCommand(90, .3, 180, driveSubsystem));
                addCommands(new DriveOnHeadingCommand(0, -.3, 170, driveSubsystem));
            }
            else if ((alliance == Alliance.Red && startingLane == AutoLane.TOP)
                || (alliance == Alliance.Blue && startingLane == AutoLane.BOTTOM)) {

                System.out.println("Balance Red/Top or Blue/Bottom");
                addCommands(new DriveOnHeadingCommand(135, .1, 1, driveSubsystem));
                addCommands(new DriveOnHeadingCommand(270, .3, 180, driveSubsystem));
                addCommands(new DriveOnHeadingCommand(0, -.3, 170, driveSubsystem));
            }

            else {
                // Start in the Middle
                System.out.println("Balance Mid");
                addCommands(new DriveOnHeadingCommand(0, -.3, 170, driveSubsystem));
            }
        }

        // Balance on the platform
        addCommands(new WaitCommand(1));
        addCommands(new BalanceCommand(driveSubsystem));
    }
}

