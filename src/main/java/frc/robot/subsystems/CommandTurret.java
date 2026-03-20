package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
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
    private static final double turretLimitRotations = 1; //! TODO: Tune turretLimitRotations
    private static final InterpolatingDoubleTreeMap hoodPositionMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap shooterPositionMap = new InterpolatingDoubleTreeMap();
    static {
        hoodPositionMap.put(1.0, 1.0);  //! TODO: Populate hoodPositionMap
        shooterPositionMap.put(1.0, 1.0); //! TODO: Populate shooterPositionMap
    }

    private static final Set<Integer> redHubTags = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
    private static final Set<Integer> blueHubTags  = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

    private static final double hoodDownPosition = 0; //! TODO: Tune hoodDownPosition

    private TalonFX turretMotor = new TalonFX(15);
    private SparkMax tunnelMotor = new SparkMax(19, SparkMax.MotorType.kBrushless);
    private TalonFX shooterMotorLeft = new TalonFX(22);
    private TalonFX shooterMotorRight = new TalonFX(20);
    private SparkMax hoodMotor = new SparkMax(21, SparkMax.MotorType.kBrushless);
    private SparkMax indexerMotor = new SparkMax(5, SparkMax.MotorType.kBrushless);

    private PIDController turretPID = new PIDController(0.1, 0, 0); //! TODO: Tune turretPID
    private PIDController hoodPID = new PIDController(0.1, 0, 0); //! TODO: Tune hoodPID

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

        BaseStatusSignal.setUpdateFrequencyForAll(50, turretMotor.getPosition(), shooterMotorLeft.getVelocity());
        turretMotor.optimizeBusUtilization();
        shooterMotorLeft.optimizeBusUtilization();
        shooterMotorRight.optimizeBusUtilization();
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
            hoodMotor.set(hoodPID.calculate(hoodMotor.getEncoder().getPosition(), hoodPositionMap.get(targetDistance)));
            tunnelMotor.set(1);
            shooterMotorLeft.set(shooterPositionMap.get(targetDistance));
            shooterMotorRight.set(-shooterPositionMap.get(targetDistance));
            indexerMotor.set(1);
        } else {
            hoodMotor.set(hoodPID.calculate(hoodMotor.getEncoder().getPosition(), hoodDownPosition));
            tunnelMotor.set(0);
            shooterMotorLeft.set(0);
            shooterMotorRight.set(0);
            indexerMotor.set(0);
        }

        SmartDashboard.putBoolean("Turret/Shooting", isShootingActive);
        SmartDashboard.putNumber("Turret/Turret Position", turretMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret/Hood Position", hoodMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Turret/Shooter Speed", shooterMotorLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Turret/Target Distance", targetDistance);
    }
}
