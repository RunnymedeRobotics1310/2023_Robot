// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants.AutoAction;
import frc.robot.Constants.AutoConstants.AutoLane;
import frc.robot.Constants.AutoConstants.Orientation;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.OiConstants;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.SystemTestCommand;
import frc.robot.commands.auto.AutonomousCommand;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.DriveModeSelector;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.commands.drive.ResetGyroPitchCommand;
import frc.robot.commands.drive.SetGyroHeadingCommand;
import frc.robot.commands.vision.SwitchVisionTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem    driveSubsystem                = new DriveSubsystem();
    private final VisionSubsystem   visionSubsystem               = new VisionSubsystem();
    private final ArmSubsystem      armSubsystem                  = new ArmSubsystem();

    // A set of choosers for autonomous patterns
    SendableChooser<AutoLane>       startingLaneChooser           = new SendableChooser<>();
    SendableChooser<GamePiece>      startingGamePieceChooser      = new SendableChooser<>();
    SendableChooser<Orientation>    startingOrientationChooser    = new SendableChooser<>();
    SendableChooser<AutoAction>     firstGamePieceScoringChooser  = new SendableChooser<>();
    SendableChooser<AutoAction>     exitZoneActionChooser         = new SendableChooser<>();
    SendableChooser<AutoAction>     secondGamePieceScoringChooser = new SendableChooser<>();
    SendableChooser<AutoAction>     balanceChooser                = new SendableChooser<>();

    // A chooser for the drive mode
    private final DriveModeSelector driveModeSelector             = new DriveModeSelector();

    // The driver's controller
    private final XboxController    driverController              = new XboxController(OiConstants.DRIVER_CONTROLLER_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Initialize all Subsystem default commands.
        driveSubsystem
            .setDefaultCommand(new DefaultDriveCommand(driverController, driveSubsystem, driveModeSelector, visionSubsystem));

        // calibrate subsystems
        calibrateVision();

        // Initialize the autonomous choosers
        initAutoSelectors();

        // Configure the button bindings
        configureButtonBindings();

        // Initialize the SmartDashboad test mode indicator
        SmartDashboard.putBoolean("Test Mode", false);
    }

    private void initAutoSelectors() {

        // FIXME: consider moving all of the choosers to their own classes.

        startingLaneChooser.setDefaultOption("Top", AutoLane.TOP);
        SmartDashboard.putData("Starting Lane", startingLaneChooser);
        startingLaneChooser.addOption("Middle", AutoLane.MIDDLE);
        startingLaneChooser.addOption("Bottom", AutoLane.BOTTOM);

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
     * Tell the vision subsystem the coordinates that it can see (on the floor).
     *
     * <pre>
     * {0, 0} corresponds to the ground directly at the front bumper in the center of the robot
     * {-10, 0} corresponds to a location against the front bumper 10cm to the left of the robot center
     * {10, 0} corresponds to a location against the front bumper 10cm to the right of the robot center
     * {10, 10} corresponds to a location 10cm away from the front bumper of the robot, 10cm to the right of center
     * </pre>
     *
     * etc.
     * These values are hard-coded in this call for now. If in the future the camera moves, they'll
     * have to
     * be re-calibrated,
     */
    private void calibrateVision() {
        // these values calibrated manually on 2023-02-02. Note - we're ignoring the top half of the
        // field of view for now.
        final double[] topLeft     = { -42.0, 78.5 };
        final double[] topRight    = { 43, 84 };
        final double[] bottomLeft  = { 13, 25 };
        final double[] bottomRight = { -13, 22.5 };

        visionSubsystem.calibrateVision(topLeft, topRight, bottomLeft, bottomRight);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // Cancel all commands on the XBox controller three lines (aka. start) button
        // NOTE: The SystemTestCommand uses the same button, so update the code in the
        // SystemTestCommand if this button changes
        new Trigger(() -> driverController.getStartButton())
            .onTrue(new CancelCommand(driveSubsystem));

        // Enter Test Mode (Start and Back pressed at the same time)
        new Trigger(() -> (driverController.getStartButton() && driverController.getBackButton()))
            .onTrue(new SystemTestCommand(driverController,
                driveSubsystem, armSubsystem));

        // Reset the Gyro heading to zero on the menu (aka. back) button
        new Trigger(() -> driverController.getBackButton())
            .onTrue(new SetGyroHeadingCommand(0, driveSubsystem)
                .andThen(new ResetGyroPitchCommand(driveSubsystem)));

        // Example using the POV to Drive on Heading at .5 speed for 50cm.
        new Trigger(() -> (driverController.getPOV() == 0))
            .onTrue(new DriveOnHeadingCommand(0, .5, 400, driveSubsystem));

        new Trigger(() -> (driverController.getPOV() == 90))
            .onTrue(new DriveOnHeadingCommand(90, .5, 400, driveSubsystem));

        new Trigger(() -> (driverController.getPOV() == 180))
            .onTrue(new DriveOnHeadingCommand(0, -.5, 400, driveSubsystem));

        new Trigger(() -> (driverController.getPOV() == 270))
            .onTrue(new DriveOnHeadingCommand(270, .5, 400, driveSubsystem));

        new Trigger(() -> (driverController.getLeftBumper()))
            .onTrue(new SwitchVisionTargetCommand(visionSubsystem));

        new Trigger(() -> (driverController.getAButton()))
            .onTrue(new BalanceCommand(driveSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return new AutonomousCommand(
            driveSubsystem,
            visionSubsystem,
            startingLaneChooser,
            startingGamePieceChooser,
            startingOrientationChooser,
            firstGamePieceScoringChooser,
            exitZoneActionChooser,
            secondGamePieceScoringChooser,
            balanceChooser);
    }
}
