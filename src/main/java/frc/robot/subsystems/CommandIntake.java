package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CommandIntake extends SubsystemBase {
    private static double pivotDownPosition = 2; //! TODO: Tune pivotDownPosition
    private static double pivotUpPosition = 1; //! TODO: Tune pivotUpPosition

    private TalonFX intakeMotor = new TalonFX(4);
    private TalonFX pivotMotor = new TalonFX(9);

    private PIDController pivotPID = new PIDController(0.05, 0, 0); //! TODO: Tune pivotPID

    private boolean isIntaking = false;

    public CommandIntake() {
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 40; //! TODO: Tune supply current limit
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.StatorCurrentLimit = 60; //! TODO: Tune stator current limit
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeMotor.getConfigurator().apply(intakeConfig);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 20; //! TODO: Tune supply current limit
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = 40; //! TODO: Tune stator current limit
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotMotor.getConfigurator().apply(pivotConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(50, pivotMotor.getPosition());
        intakeMotor.optimizeBusUtilization();
        pivotMotor.optimizeBusUtilization();
    }

    public Command toggleIntake() {
        return Commands.runOnce(() -> {
            isIntaking = !isIntaking;
        });
    }

    @Override
    public void periodic() {
        if (isIntaking) {
            pivotMotor.set(pivotPID.calculate(pivotMotor.getPosition().getValueAsDouble(), pivotDownPosition));
            intakeMotor.set(1);
        } else {
            pivotMotor.set(pivotPID.calculate(pivotMotor.getPosition().getValueAsDouble(), pivotUpPosition));
            intakeMotor.set(0);
        }

        SmartDashboard.putBoolean("Intake/Intaking", isIntaking);
        SmartDashboard.putNumber("Intake/Pivot Position", pivotMotor.getPosition().getValueAsDouble());
    }
}
