// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandTurret;
import frc.robot.subsystems.CommandClimb;
import frc.robot.subsystems.CommandIntake;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CommandIntake intake = new CommandIntake();
    public final CommandTurret turret = new CommandTurret(drivetrain);
    public final CommandClimb climb = new CommandClimb();

    private AutoFactory autoFactory = new AutoFactory(
        () -> drivetrain.getState().Pose,
        drivetrain::resetPose,
        drivetrain::followPath,
        true,
        drivetrain
    );

    private AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        autoChooser.addCmd("Simple 5m Square", this::simple5msquareAuto);
        autoChooser.addCmd("Simple 5m Relay", this::simple5mrelayAuto);
        autoChooser.addCmd("Subsystem Tests", this::subsystemtestsAuto);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.b().onTrue(intake.toggleIntake());

        joystick.a().onTrue(turret.toggleShoot());

        joystick.x().onTrue(climb.toggleClimb());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }

    public Command simple5msquareAuto() {
        return Commands.sequence(
            autoFactory.trajectoryCmd("simple_5m_square")
        );
    }

    public Command simple5mrelayAuto() {
        return Commands.sequence(
            autoFactory.trajectoryCmd("simple_5m_relay")
        );
    }

    public Command subsystemtestsAuto() {
        return Commands.sequence(
            autoFactory.trajectoryCmd("subsystem_tests")
        );
    }
}
