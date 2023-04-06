// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants.AutoAction;
import frc.robot.Constants.AutoConstants.AutoLane;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.Constants.AutoConstants.Orientation;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.Constants.OiConstants;
import frc.robot.Constants.VisionConstants.VisionTarget;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.SystemTestCommand;
import frc.robot.commands.arm.AutoTuneScore;
import frc.robot.commands.arm.CalibratePincherCommand;
import frc.robot.commands.arm.CompactCommand;
import frc.robot.commands.arm.CompactCommand_SafeForGamePiece;
import frc.robot.commands.arm.DefaultArmCommand;
import frc.robot.commands.arm.PickUpSubstationVisionCommand;
import frc.robot.commands.arm.PickupGamePieceCommand;
import frc.robot.commands.arm.ReleaseCommand;
import frc.robot.commands.arm.ScoreCommand;
import frc.robot.commands.arm.StartIntakeCommand;
import frc.robot.commands.auto.AutonomousCommand;
import frc.robot.commands.auto.DoubleDownAutoCommand;
import frc.robot.commands.auto.HumberAutoAutoCommand;
import frc.robot.commands.auto.HumberAutoOldAutoCommand;
import frc.robot.commands.auto.MiddleMayhem1AutoCommand;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.DriveToGamePieceCommand;
import frc.robot.commands.drive.ResetGyroPitchCommand;
import frc.robot.commands.drive.SetGyroHeadingCommand;
import frc.robot.commands.light.SetHoldingConeLightsCommand;
import frc.robot.commands.light.SetHoldingCubeLightsCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.commands.vision.DefaultVisionCommand;
import frc.robot.commands.vision.SetVisionTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import main.java.frc.robot.commands.light.DefaultLightCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little
 * robot logic should actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the
 * structure of the robot (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final ArmSubsystem    armSubsystem                  = new ArmSubsystem();
    private final VisionSubsystem visionSubsystem               = new VisionSubsystem();
    private final DriveSubsystem  driveSubsystem                = new DriveSubsystem(armSubsystem);
    private final LightSubsystem  lightSubsystem                = new LightSubsystem();

    // A set of choosers for autonomous patterns
    SendableChooser<AutoPattern>  autoPatternChooser            = new SendableChooser<>();
    SendableChooser<AutoLane>     startingLaneChooser           = new SendableChooser<>();
    SendableChooser<GamePiece>    startingGamePieceChooser      = new SendableChooser<>();
    SendableChooser<Orientation>  startingOrientationChooser    = new SendableChooser<>();
    SendableChooser<AutoAction>   firstGamePieceScoringChooser  = new SendableChooser<>();
    SendableChooser<AutoAction>   exitZoneActionChooser         = new SendableChooser<>();
    SendableChooser<AutoAction>   secondGamePieceScoringChooser = new SendableChooser<>();
    SendableChooser<AutoAction>   balanceChooser                = new SendableChooser<>();

    // The driver's controller
    private final OperatorInput   operatorInput                 = new OperatorInput(
        OiConstants.DRIVER_CONTROLLER_PORT, OiConstants.OPERATOR_CONTROLLER_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Initialize all Subsystem default commands.
        driveSubsystem.setDefaultCommand(
            new DefaultDriveCommand(operatorInput, driveSubsystem));

        armSubsystem.setDefaultCommand(
            new DefaultArmCommand(operatorInput, armSubsystem));

        visionSubsystem.setDefaultCommand(
            new DefaultVisionCommand(operatorInput, visionSubsystem));

        lightSubsystem.setDefaultCommand(
            new DefaultLightCommand(lightSubsystem));

        // Initialize the autonomous choosers
        initAutoSelectors();

        // Configure the button bindings
        configureButtonBindings();

        // Initialize the SmartDashboad test mode indicator
        SmartDashboard.putBoolean("Test Mode", false);
    }

    private void initAutoSelectors() {

        // FIXME: (low) consider moving all of the choosers to their own classes.
        autoPatternChooser.setDefaultOption("Double Down", AutoPattern.DOUBLE_DOWN);
        SmartDashboard.putData("Auto Pattern", autoPatternChooser);
        autoPatternChooser.addOption("Middle Mayhem", AutoPattern.MIDDLE_MAYHEM);
        autoPatternChooser.addOption("Auto Builder", AutoPattern.AUTO_BUILDER);
        autoPatternChooser.addOption("Humber Auto", AutoPattern.HUMBER_AUTO);
        autoPatternChooser.addOption("Old Humber Auto", AutoPattern.OLD_HUMBER_AUTO);


        startingLaneChooser.setDefaultOption("Substation", AutoLane.TOP);
        SmartDashboard.putData("Starting Lane", startingLaneChooser);
        startingLaneChooser.addOption("Middle", AutoLane.MIDDLE);
        startingLaneChooser.addOption("Wall", AutoLane.BOTTOM);

        startingGamePieceChooser.setDefaultOption("Cone", GamePiece.CONE);
        SmartDashboard.putData("Starting Game Piece", startingGamePieceChooser);
        startingGamePieceChooser.addOption("Cube", GamePiece.CUBE);

        startingOrientationChooser.setDefaultOption("Face Field", Orientation.FACE_FIELD);
        SmartDashboard.putData("Starting Orientation", startingOrientationChooser);
        startingOrientationChooser.addOption("Face Grid", Orientation.FACE_GRID);

        firstGamePieceScoringChooser.setDefaultOption("Bottom", AutoAction.SCORE_BOTTOM);
        SmartDashboard.putData("Score First Auto Piece", firstGamePieceScoringChooser);
        firstGamePieceScoringChooser.addOption("Middle", AutoAction.SCORE_MIDDLE);
        firstGamePieceScoringChooser.addOption("Top", AutoAction.SCORE_TOP);

        exitZoneActionChooser.setDefaultOption("Pick up Cube", AutoAction.PICK_UP_CUBE);
        SmartDashboard.putData("Exit Zone Action", exitZoneActionChooser);
        exitZoneActionChooser.addOption("Pick up Cone", AutoAction.PICK_UP_CONE);
        exitZoneActionChooser.addOption("Leave Zone (no Cube)", AutoAction.EXIT_ZONE);
        exitZoneActionChooser.addOption("Do nothing", AutoAction.DO_NOTHING);

        secondGamePieceScoringChooser.setDefaultOption("Top", AutoAction.SCORE_TOP);
        SmartDashboard.putData("Score Second Auto Piece", secondGamePieceScoringChooser);
        secondGamePieceScoringChooser.addOption("Middle", AutoAction.SCORE_MIDDLE);
        secondGamePieceScoringChooser.addOption("Bottom", AutoAction.SCORE_BOTTOM);
        secondGamePieceScoringChooser.addOption("Do not score piece", AutoAction.DO_NOTHING);

        balanceChooser.setDefaultOption("Balance", AutoAction.BALANCE);
        SmartDashboard.putData("Balance", balanceChooser);
        balanceChooser.addOption("Do not balance", AutoAction.DO_NOTHING);

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or
     * one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and
     * then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // Cancel all commands on the XBox controller three lines (aka. start) button
        // NOTE: The SystemTestCommand uses the same button, so update the code in the
        // SystemTestCommand if this button changes
        new Trigger(() -> operatorInput.isCancel())
            .onTrue(new CancelCommand(driveSubsystem, armSubsystem, visionSubsystem));

        // Enter Test Mode (Start and Back pressed at the same time)
        new Trigger(() -> (operatorInput.isToggleTestMode()))
            .onTrue(
                new SystemTestCommand(operatorInput,
                    driveSubsystem, armSubsystem, visionSubsystem));

        // Reset the Gyro heading to zero on the menu (aka. back) button
        new Trigger(() -> operatorInput.isGyroReset())
            .onTrue(new SetGyroHeadingCommand(0, driveSubsystem)
                .andThen(new ResetGyroPitchCommand(driveSubsystem)));

        // Compact the robot (safe for game piece)
        new Trigger(() -> operatorInput.isCompact())
            .onTrue(new CompactCommand_SafeForGamePiece(armSubsystem));

        // scoring (a/b/y/x)
        new Trigger(() -> (operatorInput.isHigh()))
            .onTrue(new ScoreCommand(ScoringRow.TOP, armSubsystem)
                .alongWith(new SetVisionTargetCommand(VisionTarget.POST_HIGH, visionSubsystem)));

        new Trigger(() -> (operatorInput.isMid()))
            .onTrue(new ScoreCommand(ScoringRow.MIDDLE, armSubsystem));

        new Trigger(() -> (operatorInput.isLow()))
            .onTrue(new ScoreCommand(ScoringRow.BOTTOM, armSubsystem));

        new Trigger(() -> (operatorInput.autoTune()))
            .onTrue(new AutoTuneScore(ScoringRow.TOP, visionSubsystem, driveSubsystem));

        new Trigger(() -> (operatorInput.isDrop()))
            .onTrue(new ReleaseCommand(armSubsystem));

        // grab things
        new Trigger(() -> (operatorInput.isPickUpCone()))
            .onTrue(new StartIntakeCommand(operatorInput, armSubsystem, visionSubsystem));

        new Trigger(() -> (operatorInput.isPickUpCube()))
            .onTrue(new StartIntakeCommand(operatorInput, armSubsystem, visionSubsystem));

        new Trigger(() -> (operatorInput.balance()))
            .onTrue(new BalanceCommand(driveSubsystem));

        new Trigger(() -> (operatorInput.calibratePincher()))
            .onTrue(new CalibratePincherCommand(armSubsystem)
                .andThen(new CompactCommand(armSubsystem)));

        new Trigger(() -> (operatorInput.isVisionSubstationConePickup()))
            .onTrue(new PickUpSubstationVisionCommand(operatorInput, armSubsystem, driveSubsystem, visionSubsystem));

        new Trigger(() -> (operatorInput.isCameraViewHigh()))
            .onTrue(new SetVisionTargetCommand(Constants.VisionConstants.VisionTarget.FIELD, visionSubsystem));

        new Trigger(() -> (operatorInput.isCameraViewLow()))
            .onTrue(new SetVisionTargetCommand(Constants.VisionConstants.VisionTarget.CUBE_GROUND, visionSubsystem));

        new Trigger(() -> (operatorInput.isPickUpConeVision()))
            .onTrue(new StartIntakeCommand(GamePiece.CUBE, armSubsystem, visionSubsystem)
                .deadlineWith(new WaitCommand(.5)
                    .andThen(
                        new DriveToGamePieceCommand(VisionTarget.CUBE_GROUND, .3, driveSubsystem, visionSubsystem, armSubsystem)))
                .andThen(new PickupGamePieceCommand(GamePiece.CUBE, operatorInput, armSubsystem)));

        new Trigger(() -> (armSubsystem.getHeldGamePiece() == GamePiece.CONE))
            .whileTrue(new SetHoldingConeLightsCommand(lightSubsystem));

        new Trigger(() -> (armSubsystem.getHeldGamePiece() == GamePiece.CUBE))
            .whileTrue(new SetHoldingCubeLightsCommand(lightSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        switch (autoPatternChooser.getSelected()) {

        case OLD_HUMBER_AUTO:
            return new HumberAutoOldAutoCommand(
                driveSubsystem,
                armSubsystem,
                visionSubsystem,
                startingLaneChooser);

        case HUMBER_AUTO:
            return new HumberAutoAutoCommand(
                driveSubsystem,
                armSubsystem,
                visionSubsystem,
                startingLaneChooser);

        case DOUBLE_DOWN:
            return new DoubleDownAutoCommand(
                driveSubsystem,
                armSubsystem,
                visionSubsystem,
                startingLaneChooser);

        case MIDDLE_MAYHEM:
            return new MiddleMayhem1AutoCommand(
                driveSubsystem,
                armSubsystem,
                visionSubsystem);

        case AUTO_BUILDER:

            return new AutonomousCommand(
                driveSubsystem,
                armSubsystem,
                visionSubsystem,
                startingLaneChooser,
                startingGamePieceChooser,
                startingOrientationChooser,
                firstGamePieceScoringChooser,
                exitZoneActionChooser,
                secondGamePieceScoringChooser,
                balanceChooser);

        default:
            // If the chooser did not work, then do nothing as the default auto.
            return new InstantCommand();

        }

    }
}
