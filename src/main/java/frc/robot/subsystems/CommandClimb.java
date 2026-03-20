package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CommandClimb extends SubsystemBase {
    private static double climbDownPosition = 0; //! TODO: Tune climbDownPosition
    private static double climbUpPosition = 4; //! TODO: Tune climbUpPosition
    
    private TalonFX climbMotor = new TalonFX(11);
    
    private PIDController climbPID = new PIDController(0.1, 0, 0); //! TODO: Tune climbPID

    private boolean isClimbUp = false;

    public CommandClimb() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 40; //! TODO: Tune supply current limit
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60; //! TODO: Tune stator current limit
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        climbMotor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(50, climbMotor.getPosition());
        climbMotor.optimizeBusUtilization();
    }

    public Command toggleClimb() {
        return Commands.runOnce(() -> {
            isClimbUp = !isClimbUp;
        });
    }

    @Override
    public void periodic() {
        if (isClimbUp) {
            climbMotor.set(climbPID.calculate(climbMotor.getPosition().getValueAsDouble(), climbUpPosition));
        } else {
            climbMotor.set(climbPID.calculate(climbMotor.getPosition().getValueAsDouble(), climbDownPosition));
        }

        SmartDashboard.putBoolean("Climb/Climb Up", isClimbUp);
        SmartDashboard.putNumber("Climb/Climb Position", climbMotor.getPosition().getValueAsDouble());
    }
}
