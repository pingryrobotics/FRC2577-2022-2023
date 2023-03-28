// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// commented because i don't like seeing yellow dots on my sidebar - christian
//import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
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
import frc.robot.commands.autos.AutoBalanceAuto;
import frc.robot.commands.autos.DoNothingAuto;
import frc.robot.commands.autos.MoveForwardAuto;
import frc.robot.commands.autos.OnePieceParkAuto;
import frc.robot.commands.arm_commands.*;
import frc.robot.commands.claw_commands.*;
// import frc.robot.commands.drive_commands.*;
import frc.robot.commands.shoulder_commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shoulder;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/*
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {

    // The robot's subsystems
    // private final Drive m_robotDrive = new Drive();
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Arm m_arm = new Arm(new CANSparkMax(MechanismConstants.kArmID, MotorType.kBrushless));
    // private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private final DoubleSolenoid m_DoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    private final Claw m_claw = new Claw(m_DoubleSolenoid);
    private final Shoulder m_shoulder = new Shoulder(new CANSparkMax(MechanismConstants.kShoulderID, MotorType.kBrushless));



    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    Joystick m_driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
    // Joystick m_driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    SendableChooser<Integer> side_chooser = new SendableChooser<>();

    private boolean leftJoystickPressed = false;
    private boolean rightJoystickPressed = false;
    private boolean driveOn = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        CameraServer.startAutomaticCapture();
        
        // CameraServer.

        side_chooser.setDefaultOption("Red Left", 0);
        side_chooser.addOption("Red Center", 1);
        side_chooser.addOption("Red Right", 2);
        side_chooser.addOption("Blue Left", 3);
        side_chooser.addOption("Blue Center", 4);
        side_chooser.addOption("Blue Right", 5);

        SmartDashboard.putData("Side", side_chooser);
        
                // Configure default commands
        // m_robotDrive.setDefaultCommand(
        //     // The left stick controls translation of the robot.
        //     // Turning is controlled by the X axis of the right stick.
            
        //     new RunCommand(
        //         () -> m_robotDrive.drive(
        //             -MathUtil.applyDeadband(m_driverJoystick.getY() * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.3 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
        //             -MathUtil.applyDeadband(m_driverJoystick.getX() * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.3 : 1), OIConstants.kDriveDeadband),
        //             -MathUtil.applyDeadband(m_driverJoystick.getTwist() * 0.7 * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.3 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
        //             false, true),
        //             m_robotDrive));
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY() * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.3 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX() * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.3 : 1), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX() * 0.7 * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.3 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
                    false, true),
                    m_robotDrive));
            
        
        // // Add commands to Autonomous Sendable Chooser
        m_chooser.setDefaultOption("Do Nothing", new DoNothingAuto());
        m_chooser.addOption("One Piece Park Auto", new OnePieceParkAuto(m_robotDrive, m_arm, m_claw, m_shoulder, side_chooser, true, true));
        m_chooser.addOption("One Piece Auto", new OnePieceParkAuto(m_robotDrive, m_arm, m_claw, m_shoulder, side_chooser, true, false));
        m_chooser.addOption("Park Auto", new OnePieceParkAuto(m_robotDrive, m_arm, m_claw, m_shoulder, side_chooser, false, true));
        m_chooser.addOption("Move Forward Auto", new MoveForwardAuto(m_robotDrive, m_claw, m_shoulder, m_arm, side_chooser, false));
        m_chooser.addOption("Place and Move Forward Auto", new MoveForwardAuto(m_robotDrive, m_claw, m_shoulder, m_arm, side_chooser, true));
        m_chooser.addOption("Auto Balance Auto", new AutoBalanceAuto(m_robotDrive, m_claw, m_shoulder, m_arm, side_chooser, false));
        m_chooser.addOption("Place and Auto Balance Auto", new AutoBalanceAuto(m_robotDrive, m_claw, m_shoulder, m_arm, side_chooser, true));

        
        // Put the chooser on the dashboard
        SmartDashboard.putData("Auto mode", m_chooser);
    }

    public void resetEncodersAndGyro() {
        m_arm.resetEncoder();
        m_shoulder.resetEncoder();
        m_robotDrive.zeroHeading();
    }

    public void calibrateGyro() {
        m_robotDrive.m_gyro.calibrate();
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

         new JoystickButton(m_driverJoystick, 11).whileTrue(
            new RunCommand(() ->
            m_robotDrive.setX(), m_robotDrive));
        
        new JoystickButton(m_driverJoystick, 5).onTrue(
            new RunCommand(() ->
            m_robotDrive.forwardMode())
        );

        new JoystickButton(m_driverJoystick,3).onTrue(
            new RunCommand(() ->
            m_robotDrive.reverseMode())
        );

        // new JoystickButton(m_driverJoystick, 2).whileTrue(
        //     new RunCommand(() ->
        //     m_robotDrive.slowModeOn(), m_robotDrive));
        //     new JoystickButton(m_driverJoystick, 2).whileFalse(
        //         new RunCommand(() ->
        //         m_robotDrive.slowModeOff(), m_robotDrive));
            

        // // setX (brake robot)
        // m_driverController.y().whileTrue(new RunCommand(
        //     () -> m_robotDrive.setX(),
        //     m_robotDrive));

        // m_driverController.a().whileTrue(new RunCommand(
        //     () -> m_robotDrive.m_frontLeft.setDriveSpeed(0.1)
        // ));

        // m_driverController.b().whileTrue(new RunCommand(
        //     () -> m_robotDrive.m_frontLeft.setTurnSpeed(0.1)
        // ));


        // slow mode
        // m_driverController.leftTrigger().onTrue(new RunCommand(
            // () -> m_robotDrive.toggleSlowMode(),
                    // m_robotDrive));
        // reverse mode
        // m_driverController.rightTrigger().onTrue(new RunCommand(
            // () -> m_robotDrive.toggleReverseMode(),
                    // m_robotDrive));

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

        //  m_operatorController.leftBumper().onTrue(new RunCommand(
        //     () -> m_claw.enableCompressor()
        // ));

        // m_operatorController.rightBumper().onTrue(new RunCommand(
        //     () -> m_claw.disableCompressor()
        // ));

        // extend arm fully (level 3)
        m_operatorController.y().onTrue(new ArmToHigh(m_arm));
        // extend arm to level 2 height
        m_operatorController.x().onTrue(new ArmToMid(m_arm));
        // extend arm to level 1 height
        m_operatorController.b().onTrue(new ArmToLow(m_arm));
        // retract arm fully
        m_operatorController.a().onTrue(new ArmToIn(m_arm));
        // reset arm encoder
        m_operatorController.start().onTrue(new RunCommand(
            () -> m_arm.resetEncoder()
        ));

        // m_operatorController.leftTrigger().onTrue(new RunCommand(
        //     () -> m_arm.toggleLimit(false)
        // ));

        // m_operatorController.leftTrigger().onFalse(new RunCommand(
        //     () -> m_arm.toggleLimit(true)
        // ));

        // CLAW COMMANDS
        // toggle claw
        // m_operatorController.leftBumper().onTrue(new ClawToggle(m_claw));

        // m_operatorController.rightBumper().onTrue(new RunCommand(
        //     () -> m_claw.toggleClawState()
        // ));

        // m_operatorController.leftBumper().onTrue(new RunCommand(
        //     () -> m_claw.open()
        // ));

        // m_operatorController.rightBumper().onTrue(new RunCommand(
        //     () -> m_claw.close()
        // ));

        m_operatorController.leftTrigger().onTrue(new RunCommand(() -> m_claw.stop()));

        // SHOULDER COMMANDS

        // move shoulder to level 3
        m_operatorController.pov(0).onTrue(new ShoulderToHigh(m_shoulder));
        // move shoulder to level 2
        m_operatorController.pov(90).onTrue(new ShoulderToMid(m_shoulder));
        // move shoulder to level 1
        m_operatorController.pov(270).onTrue(new ShoulderToLow(m_shoulder));
        // extend shoulder to vertical down
        m_operatorController.pov(180).onTrue(new ShoulderToIn(m_shoulder));
        // reset shoulder encoder
        m_operatorController.back().onTrue(new ShoulderEncoderReset(m_shoulder));

        m_operatorController.rightTrigger().onTrue(new RunCommand(
            () -> m_arm.toggleArmLimit()
        ));

        SmartDashboard.putNumber("Arm Position (ticks)", m_arm.getArmPosition());
        SmartDashboard.putNumber("Shoulder Position (ticks)", m_shoulder.getShoulderPosition());
    }

    // public void driveControl() {
    //     if ((Math.abs(m_driverController.getLeftX()) > 0.1) || (Math.abs(m_driverController.getLeftY()) > 0.1) || (Math.abs(m_driverController.getRightX()) > 0.1)) {
    //         m_robotDrive.drive(
    //             -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
    //             false, true);
    //         driveOn = true;
    //     } else {
    //         if (driveOn) {
    //             // m_robotDrive.drive(0, 0, 0, true, true);
    //             m_robotDrive.stop();
    //         }
    //         driveOn = false;
    //     }
    // }

    public void driveControl() {

    
        // // Configure default commands
        // m_robotDrive.setDefaultCommand(
        //     // The left stick controls translation of the robot.
        //     // Turning is controlled by the X axis of the right stick.
            
        //     new RunCommand(
        //         () -> m_robotDrive.drive(
        //             -MathUtil.applyDeadband(m_driverJoystick.getY() * driveSpeed, OIConstants.kDriveDeadband),
        //             -MathUtil.applyDeadband(m_driverJoystick.getX() * driveSpeed, OIConstants.kDriveDeadband),
        //             -MathUtil.applyDeadband(m_driverJoystick.getTwist() * driveSpeed, OIConstants.kDriveDeadband),
        //             true, true),
        //         m_robotDrive));
            
      }

    //   public void enableLimit() {
    //         m_arm.enableLimit();
    //   }



    public void containerPeriodic() {

        // ARM COMMANDS
        // while held, extend/retract the arm
        if (Math.abs(m_operatorController.getRightY()) > 0.1) {
            m_arm.setArmSpeed(m_operatorController.getRightY());
            rightJoystickPressed = true;
        } else {
            // joystick was just pressed -- means we're transitioning from arm extension to stop
            if (rightJoystickPressed) {
                m_arm.setArmSpeed(0);
                // m_arm.stop();
            }
            rightJoystickPressed = false;
        }

        // while held, extend/retract the shoulder
        if (Math.abs(m_operatorController.getLeftY()) > 0.1) {
            m_shoulder.setShoulderSpeed(m_operatorController.getLeftY() * MechanismConstants.kShoulderSpeed);
            leftJoystickPressed = true;
        } else {
            // joystick was just pressed -- means we're transitioning from shoulder extension to stop
            if (leftJoystickPressed) {
                m_shoulder.setShoulderSpeed(0);
                // m_shoulder.stop();
            }
            leftJoystickPressed = false;
        }

        if (m_operatorController.getHID().getLeftBumperPressed()) {
            m_claw.open();
        } else if (m_operatorController.getHID().getRightBumperPressed()) {
            m_claw.close();
        }

        if (m_driverJoystick.getRawButtonPressed(2) || m_driverController.getHID().getLeftBumperPressed()) {
            m_robotDrive.slowModeOn();
        } else if (m_driverJoystick.getRawButtonReleased(2) || m_driverController.getHID().getLeftBumperReleased()) {
            m_robotDrive.slowModeOff();
        }

       if (m_driverController.getHID().getRightBumperPressed()) {
            m_robotDrive.reverseMode();
        } else if (m_driverController.getHID().getRightBumperReleased()) {
            m_robotDrive.forwardMode();
        } 
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected().withTimeout(14.5);
    }
}
