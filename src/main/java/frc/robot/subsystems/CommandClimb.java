package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CommandClimb extends SubsystemBase {
    private static double climbDownPosition = 0; //! TODO: Tune climbDownPosition
    private static double climbUpPosition = 26.9; //! TODO: Tune climbUpPosition
    
    private TalonFX climbMotor = new TalonFX(11);
    
    private PIDController climbPID = new PIDController(0.1, 0, 0); //! TODO: Tune climbPID

    private boolean isClimbUp = false;

    public CommandClimb() {
        BaseStatusSignal.setUpdateFrequencyForAll(50, climbMotor.getPosition());
        climbMotor.optimizeBusUtilization();

        SmartDashboard.putBoolean("Climb/Tune", false);
        SmartDashboard.putNumber("Climb/Tune Climb Position", 0);
    }

    public Command toggleClimb() {
        return Commands.runOnce(() -> {
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
            climbMotor.set(climbPID.calculate(climbMotor.getPosition().getValueAsDouble(), climbDownPosition));
        }

        SmartDashboard.putBoolean("Climb/Climb Up", isClimbUp);
        SmartDashboard.putNumber("Climb/Climb Position", climbMotor.getPosition().getValueAsDouble());
    }
}
