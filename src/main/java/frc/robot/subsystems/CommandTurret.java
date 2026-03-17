package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class CommandTurret extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;

    private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private static final double turretLimitRotations = 5; //! TODO: Tune turretLimitRotations
    private static final InterpolatingDoubleTreeMap hoodPositionMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap shooterPositionMap = new InterpolatingDoubleTreeMap();
    static {
        hoodPositionMap.put(1.0, 1.0);  //! TODO: Populate hoodPositionMap
        shooterPositionMap.put(1.0, 1.0); //! TODO: Populate shooterPositionMap
    }

    private TalonFX turretMotor = new TalonFX(18);
    private SparkMax tunnelMotor = new SparkMax(19, SparkMax.MotorType.kBrushless);
    private TalonFX shooterMotorLeft = new TalonFX(22);
    private TalonFX shooterMotorRight = new TalonFX(20);
    private SparkMax hoodMotor = new SparkMax(21, SparkMax.MotorType.kBrushless);

    private PIDController turretPID = new PIDController(0.1, 0, 0); //! TODO: Tune turretPID
    private PIDController hoodPID = new PIDController(0.1, 0, 0); //! TODO: Tune hoodPID

    private double targetDistance = 0;

    public CommandTurret(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = turretLimitRotations;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -turretLimitRotations;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turretMotor.getConfigurator().apply(config);
    }

    public Command run() {
        return Commands.run(() -> {
            tunnelMotor.set(1); //! TODO: Tune tunnelMotor direction
            shooterMotorLeft.set(shooterPositionMap.get(targetDistance));
            shooterMotorRight.set(-shooterPositionMap.get(targetDistance));
        }, this);
    }

    public Command idle() {
        return Commands.run(() -> {
            tunnelMotor.set(0);
            shooterMotorLeft.set(0);
            shooterMotorRight.set(0);
        }, this);
    }

    public Command shooterTest(double speed) {
        return Commands.run(() -> {
            shooterMotorLeft.set(speed);
            shooterMotorRight.set(-speed);
        }, this);
    }

    @Override
    public void periodic() {
        super.periodic();

        poseEstimate();
        trackTarget();

        targetDistance = fieldLayout.getTagPose(DriverStation.getAlliance().orElseGet(() -> DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? 10 : 26).get().getTranslation().toTranslation2d().getDistance(drivetrain.getState().Pose.getTranslation());
        hoodMotor.set(hoodPID.calculate(hoodMotor.getEncoder().getPosition(), hoodPositionMap.get(targetDistance)));
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
