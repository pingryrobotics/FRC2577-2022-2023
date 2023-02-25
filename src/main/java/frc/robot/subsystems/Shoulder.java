/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

public class Shoulder extends SubsystemBase {
    private CANSparkMax shoulderMotor;
	private double speed = 0;
	private double desiredPosition = 0;
	private boolean positionMode = false;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Shoulder(CANSparkMax shoulderMotor) {
		this.shoulderMotor = shoulderMotor;
	}

	@Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (positionMode) {
			double shoulderPos = shoulderMotor.getEncoder().getPosition();
            if (Math.abs(shoulderPos - desiredPosition) < Constants.MechanismConstants.kShoulderPositionTolerance) shoulderMotor.set(0);
            else if (shoulderPos > desiredPosition) shoulderMotor.set(-Constants.MechanismConstants.kShoulderSpeed);
            else if (shoulderPos < desiredPosition) shoulderMotor.set(Constants.MechanismConstants.kShoulderSpeed);
    	}
        else shoulderMotor.set(speed);
    }
    
    public void moveShoulderDirection(int direction) {
        shoulderMotor.set(direction * Constants.MechanismConstants.kShoulderSpeed);
    }

    public void setShoulderPosition(int desiredPosition) {
        this.desiredPosition = desiredPosition;
        positionMode = true;
    }

	public double getShoulderPosition() {
		return shoulderMotor.getEncoder().getPosition();
	}

    public void setShoulderSpeed(double speed) {
        this.speed = speed;
        positionMode = false;
    }

    public void stop() {
        shoulderMotor.set(0);
    }
}