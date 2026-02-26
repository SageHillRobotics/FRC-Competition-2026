package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class CommandTurret extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;

    private TalonFX turretMotor = new TalonFX(16);

    public CommandTurret(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        super.periodic();

        poseEstimate();
        trackTarget();
    }

    public void poseEstimate() {
        LimelightHelpers.SetRobotOrientation("limelight", LimelightHelpers.getIMUData("limelight").Yaw, 0, 0, 0, 0, 0);
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        drivetrain.addVisionMeasurement(new Pose2d(poseEstimate.pose.getTranslation(), drivetrain.getPigeon2().getRotation2d()), poseEstimate.timestampSeconds);
    }

    public void trackTarget() {
        if (LimelightHelpers.getFiducialID("limelight") != 10 && LimelightHelpers.getFiducialID("limelight") != 26) {
            turretMotor.set(0);
            return;
        }
        double targetVelocity = LimelightHelpers.getTX("limelight") * -0.035;

        turretMotor.set(targetVelocity);
    }
}
