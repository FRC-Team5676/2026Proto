// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AutonManager;
import frc.robot.subsystems.FuelIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(Constants.MaxLinearRate);

    private final AutonManager autonManager = new AutonManager();
    private final CommandJoystick driver = new CommandJoystick(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final FuelIntakeSubsystem intake = new FuelIntakeSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final DriverContainer driverContainer = new DriverContainer(driver);

    public final SwerveSubsystem drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    // private final SendableChooser<Command> autoChooser;

     public RobotContainer() {
        addAutonomousChoices();
        autonManager.displayChoices();
    
        configureBindings();
    }
        
    public Command getAutonomousCommand() {
        return autonManager.getSelected();
    }

    private void addAutonomousChoices() {
        autonManager.addDefaultOption("Middle To Side", AutoCommands.moveMiddleToSide());
        autonManager.addOption("Left", AutoCommands.moveLeft());
        autonManager.addOption("Right", AutoCommands.moveRight());
        
      }
    
    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        // Drivetrain will execute this command periodically
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> driveField
                        .withVelocityX(driverContainer.getY())
                        .withVelocityY(driverContainer.getX())
                        .withRotationalRate(driverContainer.getTwist())));

        drivetrain.registerTelemetry(logger::telemeterize); 
        
        // Robot centric driving
        driver.button(12).whileTrue(
                drivetrain.applyRequest(() -> driveRobot
                        .withVelocityX(driverContainer.getY())
                        .withVelocityY(driverContainer.getX())
                        .withRotationalRate(driverContainer.getTwist())));

        // Auto-align to target when button held
        driver.button(4).whileTrue(
                drivetrain.applyRequest(() -> driveField
                        .withVelocityX(driverContainer.getY())
                        .withVelocityY(driverContainer.getX())
                        .withRotationalRate(driverContainer.getVisionTwist())));

        // Reset the field-centric heading
        driver.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driver.button(3).onTrue(intake.runOnce(() -> intake.activateIntake()));
        driver.button(3).onFalse(intake.runOnce(() -> intake.stopIntake()));

        driver.button(1).onTrue(shooter.runOnce(() -> shooter.activateShooter()));
        driver.button(1).onFalse(shooter.runOnce(() -> shooter.stopShooter()));

    }
}
