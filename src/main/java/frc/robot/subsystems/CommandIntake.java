package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CommandIntake extends SubsystemBase {
    private static double pivotDownPosition = 0.1; //! TODO: Tune pivotDownPosition
    private static double pivotUpPosition = 0; //! TODO: Tune pivotUpPosition

    private TalonFX intakeMotor = new TalonFX(14);
    private TalonFX pivotMotor = new TalonFX(15);

    private PIDController pivotPID = new PIDController(0.1, 0, 0); //! TODO: Tune pivotPID

    public Command run() {
        return Commands.run(() -> {
            pivotMotor.set(pivotPID.calculate(pivotMotor.getPosition().getValueAsDouble(), pivotDownPosition));
            intakeMotor.set(-1);
        }, this);
    }

    public Command idle() {
        return Commands.run(() -> {
            pivotMotor.set(pivotPID.calculate(pivotMotor.getPosition().getValueAsDouble(), pivotUpPosition));
            intakeMotor.set(0);
        }, this);
    }
}
