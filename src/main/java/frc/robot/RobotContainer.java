// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// commented because i don't like seeing yellow dots on my sidebar - christian
//import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
//import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MechanismConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.autos.DoNothingAuto;
import frc.robot.commands.autos.OnePieceParkAuto;
import frc.robot.commands.arm_commands.*;
import frc.robot.commands.claw_commands.*;
import frc.robot.commands.drive_commands.*;
import frc.robot.commands.shoulder_commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shoulder;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/*
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
    // The robot's subsystems
    private final Drive m_robotDrive = new Drive();
    private final Arm m_arm = new Arm(new CANSparkMax(MechanismConstants.kArmID, MotorType.kBrushless));
    private final Claw m_claw = new Claw(new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2));
    private final Shoulder m_shoulder = new Shoulder(new CANSparkMax(MechanismConstants.kShoulderID, MotorType.kBrushless));

    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    // Joystick m_driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    SendableChooser<Integer> side_chooser = new SendableChooser<>();

    private boolean leftJoystickPressed = false;
    private boolean rightJoystickPressed = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY() * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX() * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX() * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
                    true, true),
                        m_robotDrive));
        
        // Add commands to Autonomous Sendable Chooser
        m_chooser.setDefaultOption("One Piece Park Auto", new OnePieceParkAuto(m_robotDrive, m_arm, m_claw, m_shoulder, side_chooser, true, true));
        m_chooser.addOption("One Piece Auto", new OnePieceParkAuto(m_robotDrive, m_arm, m_claw, m_shoulder, side_chooser, true, false));
        m_chooser.addOption("Park Auto", new OnePieceParkAuto(m_robotDrive, m_arm, m_claw, m_shoulder, side_chooser, false, true));
        m_chooser.addOption("Do Nothing", new DoNothingAuto());

        side_chooser.setDefaultOption("Red Left", 0);
        side_chooser.addOption("Red Center", 1);
        side_chooser.addOption("Red Right", 2);
        side_chooser.addOption("Blue Left", 3);
        side_chooser.addOption("Blue Center", 4);
        side_chooser.addOption("Blue Right", 5);

        // Put the chooser on the dashboard
        SmartDashboard.putData("Auto mode", m_chooser);
        SmartDashboard.putData("Side", side_chooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        
        /*
        DRIVER CONTROLLER
        */

        // setX (brake robot)
        m_driverController.y().onTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
        // slow mode
        m_driverController.leftTrigger().onTrue(new RunCommand(
            () -> m_robotDrive.toggleSlowMode(),
                    m_robotDrive));
        // reverse mode
        m_driverController.rightTrigger().onTrue(new RunCommand(
            () -> m_robotDrive.toggleReverseMode(),
                    m_robotDrive));

        /*
        OPERATOR CONTROLLER
        */
        /**
         * Buttons:
         * 1 (trigger): toggle claw
         * 3: extend arm while held
         * 2: retract arm while held
         * 4: fully extend arm (pid)
         * 5: fully retract arm (pid)
         * : shoulder to level 3 height (pid)
         * : shoulder to pickup height (pid)
         * : shoulder to level 2 height (pid)
         * : shoulder to level 1 height (pid)
         * big joystick: front is extend shoulder while held, back is retract shoulder while held
         * speed dial upper half: regular mode for arm and shoulder
         * speed dial lower half: slow mode for arm and shoulder
         */

        // ARM COMMANDS
        // while held, extend/retract the arm
        if (Math.abs(m_operatorController.getLeftY()) > 0.1) {
            new ArmMoveSpeed(m_arm, m_operatorController.getLeftY());
            leftJoystickPressed = true;
        } else {
            // joystick was just pressed -- means we're transitioning from arm extension to stop
            if (leftJoystickPressed) {
                new ArmStop(m_arm);
            }
            leftJoystickPressed = false;
        }

        // extend arm fully (level 3)
        m_operatorController.y().onTrue(new ArmToHigh(m_arm));
        // extend arm to level 2 height
        m_operatorController.x().onTrue(new ArmToMid(m_arm));
        // extend arm to level 1 height
        m_operatorController.b().onTrue(new ArmToLow(m_arm));
        // retract arm fully
        m_operatorController.a().onTrue(new ArmToIn(m_arm));
        // reset arm encoder
        m_operatorController.start().onTrue(new ArmEncoderReset(m_arm));

        // CLAW COMMANDS
        // toggle claw
        m_operatorController.leftBumper().onTrue(new ClawToggle(m_claw));

        // SHOULDER COMMANDS
        // while held, extend/retract the shoulder
        if (Math.abs(m_operatorController.getRightY()) > 0.1) {
            new ShoulderMoveSpeed(m_shoulder, m_operatorController.getRightY());
            rightJoystickPressed = true;
        } else {
            // joystick was just pressed -- means we're transitioning from shoulder extension to stop
            if (rightJoystickPressed) {
                new ShoulderStop(m_shoulder);
            }
            rightJoystickPressed = false;
        }

        // move shoulder to level 3
        m_operatorController.pov(90).onTrue(new ShoulderToHigh(m_shoulder));
        // move shoulder to level 2
        m_operatorController.pov(180).onTrue(new ShoulderToMid(m_shoulder));
        // move shoulder to level 1
        m_operatorController.pov(0).onTrue(new ShoulderToLow(m_shoulder));
        // extend shoulder to vertical down
        m_operatorController.pov(0).onTrue(new ShoulderToIn(m_shoulder));
        // reset shoulder encoder
        m_operatorController.back().onTrue(new ShoulderEncoderReset(m_shoulder));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}
