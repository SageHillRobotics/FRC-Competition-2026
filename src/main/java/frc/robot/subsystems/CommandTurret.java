package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class CommandTurret extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;

    private static final double turretLimitRotations = 5; //! TODO: Tune turretLimitRotations

    private TalonFX turretMotor = new TalonFX(16);
    private SparkMax tunnelMotor = new SparkMax(17, SparkMax.MotorType.kBrushless);
    private TalonFX shooterMotorLeft = new TalonFX(18);
    private TalonFX shooterMotorRight = new TalonFX(19);

    private PIDController turretPID = new PIDController(0.05, 0, 0); //! TODO: Tune turretPID

    public CommandTurret(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = turretLimitRotations;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -turretLimitRotations;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turretMotor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(50, turretMotor.getPosition(), turretMotor.getVelocity());
        BaseStatusSignal.setUpdateFrequencyForAll(50, shooterMotorLeft.getPosition(), shooterMotorLeft.getVelocity());
        BaseStatusSignal.setUpdateFrequencyForAll(50, shooterMotorRight.getPosition(), shooterMotorRight.getVelocity());
        turretMotor.optimizeBusUtilization();
        shooterMotorLeft.optimizeBusUtilization();
        shooterMotorRight.optimizeBusUtilization();
    }

    public Command run() {
        return Commands.run(() -> {
            tunnelMotor.set(1); //! Tune tunnelMotor direction
            shooterMotorLeft.set(1);
            shooterMotorRight.set(-1);
        }, this);
    }

    public Command idle() {
        return Commands.run(() -> {
            tunnelMotor.set(0);
            shooterMotorLeft.set(0);
            shooterMotorRight.set(0);

        }, this);
    }

    @Override
    public void periodic() {
        super.periodic();

        poseEstimate();
        trackTarget();
    }

    public void poseEstimate() {
        LimelightHelpers.SetRobotOrientation("limelight", drivetrain.getPigeon2().getRotation2d().getDegrees(), 0, 0, 0, 0, 0);
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
    }

    public void trackTarget() {
        if (LimelightHelpers.getFiducialID("limelight") == 10 || LimelightHelpers.getFiducialID("limelight") == 26) {
            turretMotor.set(turretPID.calculate(LimelightHelpers.getTX("limelight")));
        } else {
            turretMotor.set(turretPID.calculate(LimelightHelpers.getIMUData("limelight").Yaw, drivetrain.getPigeon2().getRotation2d().getDegrees()));
        }
    }
}
