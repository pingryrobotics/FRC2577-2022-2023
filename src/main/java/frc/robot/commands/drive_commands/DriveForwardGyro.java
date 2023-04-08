package frc.robot.commands.drive_commands; //CTV

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem; //CTV

/** An example command that uses an example subsystem. */
public class DriveForwardGyro extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem m_subsystem;
    private final ADIS16448_IMU m_gyro;
    private boolean tilted = false;
    private boolean straight = false;
    private double initGyroPos;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveForwardGyro(DriveSubsystem subsystem) {
        m_subsystem = subsystem;
        m_gyro = subsystem.m_gyro;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_gyro.reset();
        initGyroPos = m_gyro.getGyroAngleY();
        m_subsystem.drive(0.3, 0, 0, false, false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if ((m_gyro.getGyroAngleY() > (initGyroPos + 10)) && !tilted) {
            // started the tilt
            m_subsystem.drive(0.1, 0, 0, false, false);
            tilted = true;
        } else if (tilted) {
            if (m_gyro.getGyroAngleY() <= (2 + initGyroPos)) {
                straight = true;
            }
        }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return tilted && straight;
    }
}