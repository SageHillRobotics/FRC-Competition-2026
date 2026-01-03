// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;

    Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("10m_forward");
    Timer autoTimer = new Timer();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        if (trajectory.isPresent()) {
            Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());
            if (initialPose.isPresent()) {
                m_robotContainer.drivetrain.resetPose(initialPose.get());
            }
        }
        autoTimer.restart();
    }

    @Override
    public void autonomousPeriodic() {
        if (trajectory.isPresent()) {
            Optional<SwerveSample> sample = trajectory.get().sampleAt(autoTimer.get(), isRedAlliance());
            if (sample.isPresent()) {
                m_robotContainer.drivetrain.followTrajectory(sample.get());
            }
        }
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
