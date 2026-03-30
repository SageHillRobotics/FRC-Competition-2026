package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

import java.util.Set;

public class CommandTurret extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;

    private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private static final double turretLimitRotations = 11; //! TODO: Tune turretLimitRotations
    private static final InterpolatingDoubleTreeMap shooterPositionMap = new InterpolatingDoubleTreeMap();
    static {
        shooterPositionMap.put(1.0, 1.0); //! TODO: Populate shooterPositionMap
    }

    private static final Set<Integer> redHubTags = Set.of(9, 10);
    private static final Set<Integer> blueHubTags  = Set.of(25, 26);

    private TalonFX turretMotor = new TalonFX(15);
    private TalonFX tunnelMotor = new TalonFX(19);
    private TalonFX shooterMotor = new TalonFX(20);
    private TalonFX indexerMotor = new TalonFX(5);

    private PIDController turretPID = new PIDController(0.5, 0, 0); //! TODO: Tune turretPID

    private double targetDistance = 0;
    private boolean isShootingActive = false;

    public CommandTurret(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = turretLimitRotations;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -turretLimitRotations;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turretMotor.getConfigurator().apply(config);
    }

    public Command toggleShoot() {
        return Commands.runOnce(() -> {
            isShootingActive = !isShootingActive;
        });
    }

    @Override
    public void periodic() {
        super.periodic();

        LimelightHelpers.SetRobotOrientation("limelight", drivetrain.getPigeon2().getRotation2d().getDegrees(), 0, 0, 0, 0, 0);
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (poseEstimate != null && poseEstimate.tagCount > 0) {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
        }

        if (isShootingActive && (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? blueHubTags : redHubTags).contains((int) LimelightHelpers.getFiducialID("limelight"))) {
            turretMotor.set(turretPID.calculate(LimelightHelpers.getTX("limelight")));
        } else {
            turretMotor.set(turretPID.calculate(turretMotor.getPosition().getValueAsDouble(), 0));
        }

        targetDistance = fieldLayout.getTagPose(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? 26 : 10).get().getTranslation().toTranslation2d().getDistance(drivetrain.getState().Pose.getTranslation());

        if (isShootingActive) {
            tunnelMotor.set(1);
            shooterMotor.set(shooterPositionMap.get(targetDistance));
            indexerMotor.set(1);
        } else {
            tunnelMotor.set(0);
            shooterMotor.set(0);
            indexerMotor.set(0);
        }

        SmartDashboard.putBoolean("Turret/Shooting", isShootingActive);
    }
}
