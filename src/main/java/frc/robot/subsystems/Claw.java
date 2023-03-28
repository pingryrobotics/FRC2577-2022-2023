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

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.commands.claw_commands.ClawStop;

public class Claw extends SubsystemBase {
    private DoubleSolenoid clawSolenoid;
	// private Solenoid singleSolenoid;
	private boolean state = false;
	private Compressor compressor;
	private Rev2mDistanceSensor colorSensor;
	private boolean autoClaw = true;
	private boolean objectExists = false;
	private boolean objectExisted = false;

    
	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Claw(DoubleSolenoid clawSolenoid, ColorSensorV3 colorSensor) {
		this.clawSolenoid = clawSolenoid;
		this.colorSensor = colorSensor;
		// this.clawSolenoid.set(DoubleSolenoid.Value.kOff);
		// this.singleSolenoid = solenoid;
		// solenoid.set(true);
		compressor = new Compressor(PneumaticsModuleType.CTREPCM);
		this.clawSolenoid.set(DoubleSolenoid.Value.kForward);
		// compressor.disable();
		// this.setDefaultCommand(new ClawStop(this));
		colorSensor.setAutomaticMode(true);
	}

	@Override
	public void periodic() {
		if (colorSensor.isRangeValid() && colorSensor.getRange() < 40) {
			objectExisted = objectExists;
			objectExists = true;
		} else {
			objectExisted = objectExists;
			objectExists = false;
		}
		// This method will be called once per scheduler run
		if (autoClaw && !objectExisted && objectExists) {
			this.close();
		}
	}

	public void toggleAutoClaw() {
		autoClaw = !autoClaw;
	}

	public void enableCompressor() {
		compressor.enableDigital();
	}

	public void disableCompressor() {
		compressor.disable();
	}

	public void stop() {
		clawSolenoid.set(DoubleSolenoid.Value.kOff);
	}
	public void close() {
		// singleSolenoid.set(true);
		// clawSolenoid.toggle();
		clawSolenoid.set(DoubleSolenoid.Value.kForward);
		state = false;
	}

	public void toggleClawState() {
		state = !state;
		clawSolenoid.toggle();
	}

	public void open() {
		// singleSolenoid.set(true);
		// clawSolenoid.toggle();
		clawSolenoid.set(DoubleSolenoid.Value.kReverse);
		state = true;
	}

	// DoubleSolenoid.toggle() is unreliable since it handles toggling away from kOff badly
	public void toggle() {
		if(state) close();
		else open();
	}
}