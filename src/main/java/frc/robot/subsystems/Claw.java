/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// DEPRECATED - DOESN'T USE SOLENOIDS
// REWRITE TO USE SOLENOIDS

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import frc.robot.commands.claw_commands.ClawStop;

public class Claw extends SubsystemBase {
    private DoubleSolenoid clawSolenoid;
	private boolean state;

    
	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Claw(DoubleSolenoid clawSolenoid) {
		this.clawSolenoid = clawSolenoid;
		this.clawSolenoid.set(DoubleSolenoid.Value.kOff);
		this.setDefaultCommand(new ClawStop(this));
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void stop() {
		clawSolenoid.set(DoubleSolenoid.Value.kOff);
	}

	public void close() {
		clawSolenoid.set(DoubleSolenoid.Value.kForward);
		state = false;
	}

	public void open() {
		clawSolenoid.set(DoubleSolenoid.Value.kReverse);
		state = true;
	}

	// DoubleSolenoid.toggle() is unreliable since it handles toggling away from kOff badly
	public void toggle() {
		if(state) close();
		else open();
	}
}