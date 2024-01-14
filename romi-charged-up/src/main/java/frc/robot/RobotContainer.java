// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.RomiDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final RomiDrivetrain m_drivetrain = new RomiDrivetrain();
    private final ExampleCommand m_autoCommand = new ExampleCommand(m_drivetrain);
    private final CommandXboxController m_controller = new CommandXboxController(1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_drivetrain.setDefaultCommand(this.getArcadeDriveCommand());

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        m_controller.b().onTrue(this.DriveUntilCommand(10));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_autoCommand;
    }


    public Command getArcadeDriveCommand() {
        // return new ArcadeDrive(
        //     m_drivetrain, () -> -m_controller.getRawAxis(1), () -> -m_controller.getRawAxis(2));
        return new RunCommand(
            () -> m_drivetrain.arcadeDrive(-m_controller.getLeftY(), m_controller.getRightX()),
            m_drivetrain
        );
    }

    public Command DriveUntilCommand(final double dist) {
        return new FunctionalCommand(
            // Reset encoders on command start
            m_drivetrain::resetEncoders,
            // Start driving forward at the start of the command
            m_drivetrain::driveForward,
            // Stop driving at the end of the command
            (interrupted) -> m_drivetrain.arcadeDrive(0, 0),
            // End the command when the robot's driven distance exceeds the desired value
            () -> m_drivetrain.getAverageEncoderDistance() >= dist,
            // Require the drive subsystem
            m_drivetrain
        );
    }
}
