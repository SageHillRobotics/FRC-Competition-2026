package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class CommandTurret extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;

    private static final double turretLimitRotations = 5; //! TODO: Tune turretLimitRotations

    private TalonFX turretMotor = new TalonFX(16);

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
        turretMotor.optimizeBusUtilization();
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
        if (poseEstimate != null && poseEstimate.tagCount > 0) {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            drivetrain.addVisionMeasurement(new Pose2d(poseEstimate.pose.getTranslation(), drivetrain.getPigeon2().getRotation2d()), poseEstimate.timestampSeconds);
        }
    }

    public void trackTarget() {
        if (LimelightHelpers.getFiducialID("limelight") == 10 || LimelightHelpers.getFiducialID("limelight") == 26) {
            turretMotor.set(turretPID.calculate(LimelightHelpers.getTX("limelight")));
        } else {
            turretMotor.set(turretPID.calculate(LimelightHelpers.getIMUData("limelight").Yaw, drivetrain.getPigeon2().getRotation2d().getDegrees()));
        }
    }
}
