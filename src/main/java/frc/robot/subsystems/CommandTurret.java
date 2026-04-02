package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

import java.util.Set;

public class CommandTurret extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    private CommandXboxController joystick;

    private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private static final InterpolatingDoubleTreeMap shooterVelocityMap = new InterpolatingDoubleTreeMap();
    static {
        shooterVelocityMap.put(1.0, 1.0); //! TODO: Populate shooterVelocityMap
    }

    private static final Set<Integer> redHubTags = Set.of(9, 10);
    private static final Set<Integer> blueHubTags  = Set.of(25, 26);
    private static final double turretLimitRotations = 11; //! TODO: Tune turretLimitRotations

    private TalonFX turretMotor = new TalonFX(15);
    private TalonFX tunnelMotor = new TalonFX(19);
    private TalonFX shooterMotor = new TalonFX(20);
    private SparkMax indexerMotor = new SparkMax(5, MotorType.kBrushless);

    private PIDController turretPID = new PIDController(0.5, 0, 0); //! TODO: Tune turretPID

    private double targetDistance = 0;
    private boolean isShootingActive = false;
    private boolean isAntistuckActive = false;
    private boolean isManualActive = false;

    private Timer timer = new Timer();

    public CommandTurret(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick) {
        this.drivetrain = drivetrain;
        this.joystick = joystick;

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
            isAntistuckActive = false;
            isManualActive = false;
            timer.restart();
        }, this);
    }

    public Command toggleManual() {
        return Commands.runOnce(() -> {
            isManualActive = !isManualActive;
            isAntistuckActive = false;
            isShootingActive = false;
            timer.restart();
        }, this);
    }

    public Command toggleAntistuck() {
        return Commands.runOnce(() -> {
            isAntistuckActive = !isAntistuckActive;
            isShootingActive = false;
            isManualActive = false;
            timer.restart();
        }, this);
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
        } else if (isManualActive) {
            if (joystick.leftBumper().getAsBoolean()) {
                turretMotor.set(-1);
            } else if (joystick.rightBumper().getAsBoolean()) {
                turretMotor.set(1);
            } else if (joystick.x().getAsBoolean()) {
                turretMotor.set(turretPID.calculate(turretMotor.getPosition().getValueAsDouble(), 0));
            } else {
                turretMotor.set(0);
            }
        } else {
            turretMotor.set(0);
        }

        targetDistance = fieldLayout.getTagPose(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? 26 : 10).get().getTranslation().toTranslation2d().getDistance(drivetrain.getState().Pose.getTranslation());

        if (isShootingActive || isManualActive) {
            shooterMotor.set(shooterVelocityMap.get(targetDistance));
            if (timer.hasElapsed(0.5)) {
                tunnelMotor.set(1);
                indexerMotor.set(1);
            }
        } else if (isAntistuckActive) {
            shooterMotor.set(-1);
            tunnelMotor.set(-1);
            indexerMotor.set(-1);
        } else {
            shooterMotor.set(0);
            tunnelMotor.set(0);
            indexerMotor.set(0);
        }

        SmartDashboard.putBoolean("Turret/Shooting", isShootingActive);
        SmartDashboard.putBoolean("Turret/Manual Mode", isManualActive);
        SmartDashboard.putBoolean("Turret/Jam Prevention", isAntistuckActive);
    }
}
