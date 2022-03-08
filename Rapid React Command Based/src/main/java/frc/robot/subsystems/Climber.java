// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parameters;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Climber extends SubsystemBase {
  
    // private Xbox gunner = RobotContainer.gunner;
    private static Climber instance = null;
    private static DoubleSolenoid bigArmSolenoid;
    private static DoubleSolenoid smallHookSolenoid;
    private static WPI_TalonFX rightClimberMotor;
    private static WPI_TalonFX leftClimberMotor;

    public Climber() {
        bigArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Parameters.BIG_ARM_DOWN_CHANNEL, Parameters.BIG_ARM_UP_CHANNEL);
        smallHookSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Parameters.SMALL_HOOK_DISENGAGE_CHANNEL, Parameters.SMALL_HOOK_ENGAGE_CHANNEL);
        rightClimberMotor = new WPI_TalonFX(Parameters.RIGHT_CLIMBER_ID);
        leftClimberMotor = new WPI_TalonFX(Parameters.LEFT_CLIMBER_ID);
        leftClimberMotor.setNeutralMode(NeutralMode.Brake);
        rightClimberMotor.setNeutralMode(NeutralMode.Brake);
    }

	public static synchronized Climber getInstance() {
		if (instance == null) {
			instance = new Climber();
		} 
		return instance;
	}

  /**
     * Runs the intake to take a ball from the field into the conveyor system
     */
    public void raiseBigArms() {
      bigArmSolenoid.set(Value.kForward);
  }

  /**
   * Runs the intake to take a ball from the field into the conveyor system
   */
  public void lowerBigArms() {
      bigArmSolenoid.set(Value.kReverse);
  }
  /**
     * Runs the intake to take a ball from the field into the conveyor system
     */
    public void engageSmallHooks() {
      smallHookSolenoid.set(Value.kForward);
  }

  /**
   * Runs the intake to take a ball from the field into the conveyor system
   * DEFAULT
   */
  public void disengageSmallHooks() {
      smallHookSolenoid.set(Value.kReverse);
  }

  /*
  climberMotorSpeedLimiter = new SlewRateLimiter(Parameters.CLIMBER_MOTOR_SPEED);
  */
  //right motor is reverse of left
  public void raiseHooks() {
    // rightClimberMotor.set(climberMotorSpeed.ySpeedRight);
    // leftClimberMotor.set(climberMotorSpeed.ySpeedRight);
    leftClimberMotor.set(-Parameters.CLIMBER_MOTOR_SPEED);
    rightClimberMotor.set(Parameters.CLIMBER_MOTOR_SPEED);
  }

  //right motor is reverse of left
  public void lowerHooks() {
    
  //if (getRightMotorEncoderCounts() < Parameters.CLIMBER_MAX_ENCODER_COUNTS || getLeftMotorEncoderCounts() < Parameters.CLIMBER_MAX_ENCODER_COUNTS) {
    //rightClimberMotor.set(-Parameters.CLIMBER_MOTOR_SPEED);
    //leftClimberMotor.set(Parameters.CLIMBER_MOTOR_SPEED);
  //} else{
    //return;
  //}
    //rightClimberMotor.set(InstantCommand(() -> armAdjustmentCmdRight));
  //  rightClimberMotor.set(climberMotorSpeed.ySpeedRight);
  //  leftClimberMotor.set(climberMotorSpeed.ySpeedLeft);
    leftClimberMotor.set(Parameters.CLIMBER_MOTOR_SPEED);
    rightClimberMotor.set(-Parameters.CLIMBER_MOTOR_SPEED);
  }

  public void rightHookControl(double speed) {
    leftClimberMotor.set(-speed);
  }

  public void leftHookControl(double speed) {
    rightClimberMotor.set(speed);
  }

  public void stopClimberMotors() {
    rightClimberMotor.set(0);
    leftClimberMotor.set(0);
  }

  public double getRightMotorEncoderCounts() {
      return rightClimberMotor.getSensorCollection().getIntegratedSensorPosition();
  }

  public double getLeftMotorEncoderCounts() {
      return leftClimberMotor.getSensorCollection().getIntegratedSensorPosition();
  }

}
