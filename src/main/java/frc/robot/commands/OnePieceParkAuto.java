package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.FieldConstants.Community;
import frc.robot.FieldConstants.Grids;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.util.trajectory.TrajectoryCommandGenerator;

/** An example command that uses an example subsystem. */
public class OnePieceParkAuto extends SequentialCommandGroup {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drive m_robotDrive;
    private final Arm m_arm;
    private final Claw m_claw;
    private final Shoulder m_shoulder;

	/**
	* One Piece Park Auto.
	* Places preloaded piece (cube) onto node and parks on a third of the charge station
	*/
    public OnePieceParkAuto(Drive drive, Arm arm, Claw claw, Shoulder shoulder, SendableChooser<Integer> side_chooser, boolean park) {
        m_robotDrive = drive;
        m_arm = arm;
        m_claw = claw;
        m_shoulder = shoulder;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_robotDrive);
        addRequirements(m_arm);
        addRequirements(m_claw);
        addRequirements(m_shoulder);

        /**
         * COORDINATE SYSTEM
         * All coordinates based on close right corner (from view of blue alliance driver station)
         * Y is movement along the same distance from driver station, left is positive
         * X is movement away and towards driver station, away is positive
         */

        // we will start right in front of a cube node (either left center or right)
        Pose2d startPose;

        double startXPos = Grids.outerX + Constants.DriveConstants.kWheelBase;

        // only waypoint: x is halfway between initial position and close edge of charging station, y is exactly in the center (will be edited)
        List<Translation2d> waypoints = List.of(new Translation2d((startXPos + Community.chargingStationInnerX)/2, Grids.nodeFirstY + 4 * Grids.nodeSeparationY));

        Pose2d endPose = new Pose2d(((Community.chargingStationInnerX + Community.chargingStationOuterX)),
        (Community.chargingStationLeftY + Community.chargingStationRightY)/2, new Rotation2d(0));

        // each y position is a cube node, nodeFirstY is the first node, nodeSeparationY is the distance between nodeSeparationY
        // startXPos is the x position of the robot (i.e. right outside the grids )
        switch (side_chooser.getSelected() % 3) {
            // left
            case 0:
                // nodeFirstY + 7*nodeSeparationY is the last node
                startPose = new Pose2d(startXPos, Grids.nodeFirstY + 7 * Grids.nodeSeparationY, new Rotation2d(0));
                waypoints.get(0).plus(new Translation2d(0, Grids.nodeSeparationY));
                // if we are on the left, we need to move the end pose to the left to make room for the other robots
                endPose = endPose.plus(
                        new Transform2d(new Translation2d(0, Community.chargingStationWidth / 3), new Rotation2d(0)));

            // center
            case 1:
                // nodeFirstY + 4*nodeSeparationY is the fourth node (indexed from 0)
                startPose = new Pose2d(startXPos, Grids.nodeFirstY + 4 * Grids.nodeSeparationY, new Rotation2d(0));
                waypoints.get(0).plus(new Translation2d(0, 0));
                // if we are in the center, we leave endpose as is

            // right
            default:
                // nodeFirstY + 1*nodeSeparationY is the first node (indexed from 0)
                startPose = new Pose2d(startXPos, Grids.nodeFirstY + 1 * Grids.nodeSeparationY, new Rotation2d(0));
                waypoints.get(0).plus(new Translation2d(0, -Grids.nodeSeparationY));
                // if we are on the right, we need to move the end pose to the right to make room for the other robots
                endPose = endPose.plus(
                        new Transform2d(new Translation2d(0, -Community.chargingStationWidth / 3), new Rotation2d(0)));
            // // blue left
            // case 3:
            //     startPose = new Pose2d(0, 0, new Rotation2d(0));
            //     waypoints = List.of(
            // // blue center
            // case 4:
            //     startPose = new Pose2d(0, 0, new Rotation2d(0));
            //     waypoints = List.of(
            // // blue right
            // case 5:
            //     startPose = new Pose2d(0, 0, new Rotation2d(0));
            //     waypoints = List.of(
        }

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

        // from the start pose, go to the waypoints, and then end at the end pose
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            startPose,
            // Pass through these two interior waypoints, making an 's' curve path
            waypoints,
            // End 3 meters straight ahead of where we started, facing forward
            endPose,
            config);

        SwerveControllerCommand toPark = TrajectoryCommandGenerator.generateCommand(traj, drive);

        addCommands(
            // new RunCommand(() -> m_shoulder.setShoulderPosition(0.5), m_shoulder),
            new WaitCommand(3.0),
            // new RunCommand(() -> m_arm.setArmPosition(0.5), m_arm),
            new WaitCommand(4.0),
            // new RunCommand(() -> m_claw.setClawPosition(0.5), m_claw),
            new WaitCommand(4.0)
        );
        if (park) {
            addCommands(
                new RunCommand(() -> m_robotDrive.resetOdometry(traj.getInitialPose())),
                toPark.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false))
            );
        }
	}
} 