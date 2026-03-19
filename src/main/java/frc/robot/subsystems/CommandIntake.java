package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CommandIntake extends SubsystemBase {
    private static double pivotDownPosition = 4; //! TODO: Tune pivotDownPosition
    private static double pivotUpPosition = 0; //! TODO: Tune pivotUpPosition

    private TalonFX intakeMotor = new TalonFX(4);
    private TalonFX pivotMotor = new TalonFX(9);

    private PIDController pivotPID = new PIDController(0.1, 0, 0); //! TODO: Tune pivotPID

    private boolean isIntaking = false;

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
