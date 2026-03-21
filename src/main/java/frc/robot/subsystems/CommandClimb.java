package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CommandClimb extends SubsystemBase {
    private static final double climbDownPosition = 0;
    private static final double climbPartialDownPosition = 1;
    private static final double climbUpPosition = 58;

    private TalonFX climbMotor = new TalonFX(11);

    private PIDController climbPID = new PIDController(0.05, 0, 0);

    private boolean isClimbUp = false;
    private double activeDownPosition = climbDownPosition;

    public CommandClimb() {
        BaseStatusSignal.setUpdateFrequencyForAll(50, climbMotor.getPosition());
        climbMotor.optimizeBusUtilization();

        SmartDashboard.putBoolean("Climb/Tune", false);
        SmartDashboard.putNumber("Climb/Tune Climb Position", 0);
    }

    public Command toggleClimb() {
        return Commands.runOnce(() -> {
            activeDownPosition = climbDownPosition;
            isClimbUp = !isClimbUp;
        });
    }

    public Command toggleClimbPartial() {
        return Commands.runOnce(() -> {
            activeDownPosition = climbPartialDownPosition;
            isClimbUp = !isClimbUp;
        });
    }

    @Override
    public void periodic() {
        boolean tuning = SmartDashboard.getBoolean("Climb/Tune", false);
        if (tuning) {
            double tunePos = SmartDashboard.getNumber("Climb/Tune Climb Position", 0);
            climbMotor.set(climbPID.calculate(climbMotor.getPosition().getValueAsDouble(), tunePos));
        } else if (isClimbUp) {
            climbMotor.set(climbPID.calculate(climbMotor.getPosition().getValueAsDouble(), climbUpPosition));
        } else {
            climbMotor.set(climbPID.calculate(climbMotor.getPosition().getValueAsDouble(), activeDownPosition));
        }

        SmartDashboard.putBoolean("Climb/Climb Up", isClimbUp);
        SmartDashboard.putNumber("Climb/Climb Position", climbMotor.getPosition().getValueAsDouble());
    }
}
