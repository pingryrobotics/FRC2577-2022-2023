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

public class Arm extends SubsystemBase {
    private CANSparkMax armMotor;
    private double desiredPosition = 0;
    private boolean positionMode = false;
    private double speed = 0;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Arm(CANSparkMax armMotor) {
        this.armMotor = armMotor;
	}

	@Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (positionMode) {
            if (Math.abs(armMotor.getEncoder().getPosition() - desiredPosition) < Constants.MechanismConstants.kArmPositionTolerance) {
                armMotor.set(0);
            } else if (armMotor.getEncoder().getPosition() > desiredPosition) {
                armMotor.set(-Constants.MechanismConstants.kArmSpeed);
            } else if (armMotor.getEncoder().getPosition() < desiredPosition) {
                armMotor.set(Constants.MechanismConstants.kArmSpeed);
            }
        } else {
            armMotor.set(speed);
        }
    }
    
    public void moveArmDirection(int direction) {
        armMotor.set(direction * Constants.MechanismConstants.kArmSpeed);
    }

    public void setArmPosition(int desiredPosition) {
        this.desiredPosition = desiredPosition;
        positionMode = true;
    }

    public void getArmPosition() {
        return armMotor.getEncoder().getPosition();
    }

    public void setArmSpeed(double speed) {
        this.speed = speed;
        positionMode = false;
    }

    public void stop() {
        armMotor.set(0);
    }
}