// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OiConstants;
import frc.robot.commands.auto.AutonomousCommand;
import frc.robot.commands.shooter.DefaultShooterCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ShootLowCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final IntakeSubsystem intakeSubsystem     = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem   = new ShooterSubsystem();


    // A chooser for autonomous commands
    SendableChooser<String> autoChooser = new SendableChooser<>();

    // The driver's controller
    private final XboxController driverController = new XboxController(OiConstants.DRIVER_CONTROLLER_PORT);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Initialize all Subsystem default commands.
        shooterSubsystem .setDefaultCommand(new DefaultShooterCommand (driverController, shooterSubsystem));

        // Initialize the autonomous chooser
        autoChooser.setDefaultOption(AutoConstants.AUTO_PATTERN_DO_NOTHING, AutoConstants.AUTO_PATTERN_DO_NOTHING);
        SmartDashboard.putData(autoChooser);
        autoChooser.addOption(AutoConstants.AUTO_PATTERN_SHOOT, AutoConstants.AUTO_PATTERN_SHOOT);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // Button map for Driver Stick
        Button shootButton = new JoystickButton(driverController, XboxController.Button.kY.value);
        Button shootLowButton = new JoystickButton(driverController, XboxController.Button.kX.value);

        // Button binding
        shootButton.whenPressed(new ShootCommand(shooterSubsystem, intakeSubsystem));
        shootLowButton.whenPressed(new ShootLowCommand(shooterSubsystem, intakeSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return new AutonomousCommand(
                intakeSubsystem,
                shooterSubsystem,
                autoChooser);

    }
}
