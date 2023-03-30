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

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.claw_commands.ClawStop;

public class Claw extends SubsystemBase {
    private DoubleSolenoid clawSolenoid;
	// private Solenoid singleSolenoid;
	private CANSparkMax wheelsMotor;
	private boolean state = false;
	private Compressor compressor;
	private ColorSensorV3 colorSensor;
	public boolean autoClaw = false;
	public boolean objectExists = false;
	public boolean objectExisted = false;
	// private int cnt = 0;
	public double wheelsSpeed = 0;

    
	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Claw(CANSparkMax wheelsMotor, DoubleSolenoid clawSolenoid, ColorSensorV3 colorSensor) {
		this.wheelsMotor = wheelsMotor;
		this.clawSolenoid = clawSolenoid;
		this.colorSensor = colorSensor;
		// this.clawSolenoid.set(DoubleSolenoid.Value.kOff);
		// this.singleSolenoid = solenoid;
		// solenoid.set(true);
		compressor = new Compressor(PneumaticsModuleType.CTREPCM);
		this.clawSolenoid.set(DoubleSolenoid.Value.kForward);
		// compressor.disable();
		// this.setDefaultCommand(new ClawStop(this));
		SmartDashboard.putBoolean("Has closed", false);
	}

	// public void enableAutomatic() {
		// colorSensor.setAutomaticMode(true);
	// }

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Color Sensor Range", colorSensor.getProximity());
		// SmartDashboard.putNumber("Color Sensor Timestamp", colorSensor.getBlue());
		SmartDashboard.putBoolean("AutoClaw (tm) On", autoClaw);
		SmartDashboard.putBoolean("Claw Is Open", state);

		if (colorSensor.getProximity() > 110) {
			objectExisted = objectExists;
			objectExists = true;
			// this.close();
		} else {
			objectExisted = objectExists;
			objectExists = false;
		}

		// SmartDashboard.putBoolean("Existed", objectExisted);
		// SmartDashboard.putBoolean("Exists", objectExists);

		// This method will be called once per scheduler run
	}

	public void setWheelsSpeed(double speed) {
		wheelsSpeed = speed;
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
	// public void toggle() {
	// 	if(state) close();
	// 	else open();
	// }
}