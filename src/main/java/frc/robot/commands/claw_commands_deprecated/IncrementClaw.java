// DEPRECATED - DOESN'T USE SOLENOIDS

package frc.robot.commands.claw_commands_deprecated;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;

/** An example command that uses an example subsystem. */
public class IncrementClaw extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Claw m_claw;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IncrementClaw(Claw claw) {
        m_claw = claw;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_claw.setClawSpeed(Constants.MechanismConstants.kClawSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("Claw position: " + m_claw.getClawPosition());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}