// DEPRECATED - DOESN'T USE SOLENOIDS

package frc.robot.commands.claw_commands_deprecated;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;

/** An example command that uses an example subsystem. */
public class CloseClawPos extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Claw m_claw;

	/**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public CloseClawPos(Claw claw) {
    	m_claw = claw;
    	// Use addRequirements() here to declare subsystem dependencies.
    	addRequirements(claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    	m_claw.setClawPosition(kClawClosedPosition);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    	System.out.println("Claw position: " + m_claw.getClawPosition());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
		m_claw.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
    	return Math.abs(m_claw.getClawPosition() - Constants.MechanismConstants.kClawClosedPosition) < Constants.MechanismConstants.kClawPositionTolerance;
    }
}