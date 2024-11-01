package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Pivot;

public class Pivotto30Deg extends Command {
    private final Pivot pivot;
    private final double targetPosition;

    public Pivotto30Deg(Pivot pivot) {
        this.pivot = pivot;
        this.targetPosition = 30; // Convert 30 degrees to position units
        addRequirements(pivot); // Declare subsystem dependencies
    }

    @Override
    public void initialize() {
        // Start by setting the target position to 30 degrees
        pivot.rotateToPosition(targetPosition);
    }

    @Override
    public void execute() {
        // Check if the IR beam break sensor is triggered
        if (pivot.isPivotBeamBroken()) {
            pivot.resetPivotEncoder(); // Zero the encoder if the beam is broken
        }
    }

    @Override
    public boolean isFinished() {
        // End the command when the pivot reaches the 30-degree setpoint
        return pivot.isAtPositionSetpoint(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Rotation to 30 degrees was interrupted");
        } else {
            System.out.println("Pivot successfully reached 30 degrees");
        }
    }
}
