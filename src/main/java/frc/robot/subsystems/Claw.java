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

public class Claw extends SubsystemBase {
    private CANSparkMax clawMotor;
	private double desiredPosition = 0;
	private double speed = 0;
	private boolean positionMode = false;

    
	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Claw(CANSparkMax clawMotor) {
		this.clawMotor = clawMotor;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
        if (positionMode) {
			double clawPos = clawMotor.getEncoder().getPosition();
            if (Math.abs(clawPos - desiredPosition) < Constants.MechanismConstants.kClawPositionTolerance) clawMotor.set(0);
			else if (clawPos > desiredPosition) clawMotor.set(-Constants.MechanismConstants.kClawSpeed);
            else if (clawPos < desiredPosition) clawMotor.set(Constants.MechanismConstants.kClawSpeed);
		} else {
			clawMotor.set(speed);
		}
	}

	public void moveClawDirection(int direction) {
        clawMotor.set(direction * Constants.MechanismConstants.kClawSpeed);
    }

	public void setClawSpeed(double speed) {
		this.speed = speed;
		positionMode = false;
	}

	public void setClawPosition(double desiredPosition) {
        this.desiredPosition = desiredPosition;
        positionMode = true;
    }

	public double getClawPosition() {
		return clawMotor.getEncoder().getPosition();
	}

	public void open() {
		this.setClawPosition(Constants.MechanismConstants.kClawOpenPosition);
	}

	public void close() {
		this.setClawPosition(Constants.MechanismConstants.kClawClosedPosition);
	}

	public void stop() {
		clawMotor.set(0);
	}
}