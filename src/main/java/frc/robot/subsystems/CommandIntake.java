package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CommandIntake extends SubsystemBase {
    private static double pivotDownPosition = 4.16;
    private static double pivotUpPosition = 0;

    private TalonFX intakeMotor = new TalonFX(14);
    private TalonFX pivotMotor = new TalonFX(15);

    private PIDController pivotPID = new PIDController(0.1, 0, 0); //! TODO: Tune pivotPID

    private boolean isPivotDown = false;

    public CommandIntake() {
        BaseStatusSignal.setUpdateFrequencyForAll(50, pivotMotor.getPosition(), pivotMotor.getVelocity());
        BaseStatusSignal.setUpdateFrequencyForAll(50, intakeMotor.getPosition(), intakeMotor.getVelocity());
        pivotMotor.optimizeBusUtilization();
        intakeMotor.optimizeBusUtilization();
    }

    public Command run() {
        return Commands.run(() -> {
            intakeMotor.set(-1);
        }, this);
    }

    public Command idle() {
        return Commands.run(() -> {
            intakeMotor.set(0);
        }, this);
    }

    public Command togglePivot() {
        return Commands.run(() -> {
            isPivotDown = !isPivotDown;
            if (isPivotDown) {
                pivotMotor.set(pivotPID.calculate(pivotMotor.getPosition().getValueAsDouble(), pivotDownPosition));
            } else {
                pivotMotor.set(pivotPID.calculate(pivotMotor.getPosition().getValueAsDouble(), pivotUpPosition));
            }
        }, this);
    }
}
