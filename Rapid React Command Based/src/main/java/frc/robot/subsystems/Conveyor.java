// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parameters;
import frc.robot.commands.Conveyor.RaiseIntake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
public class Conveyor extends SubsystemBase {
  private final TalonFX shooterMotor;
    private final VictorSPX conveyorMotor;
    private final VictorSPX intakeMotor;
    private static Conveyor instance = null;
    public DigitalInput conveyorSensor;
    private static DoubleSolenoid solenoid;
    
    public Conveyor() {
      intakeMotor = new VictorSPX(Parameters.INTAKE_MOTOR_ID);
        shooterMotor = new TalonFX(Parameters.SHOOTER_MOTOR_ID);
        conveyorMotor = new VictorSPX(Parameters.CONVEYOR_MOTOR_ID);
        conveyorSensor = new DigitalInput(Parameters.CONVEYOR_BALL_SENSOR_ID);
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Parameters.INTAKE_DOWN_CHANNEL, Parameters.INTAKE_UP_CHANNEL);
    }

	public static synchronized Conveyor getInstance() {
		if (instance == null) {
			instance = new Conveyor();
		} 
		return instance;
	}

  /**
     * Runs the intake to take a ball from the field into the conveyor system
     */
    public void raiseIntake() {
      solenoid.set(Value.kForward);
  }

  /**
   * Runs the intake to take a ball from the field into the conveyor system
   */
  public void lowerIntake() {
      solenoid.set(Value.kReverse);
  }

  /**
   * Runs the intake to take a ball from the field into the conveyor system
   */
  public void intakeBall() {
      intakeMotor.set(ControlMode.PercentOutput, Parameters.INTAKE_MOTOR_SPEED);
  }

  /**
   * Runs the intake in the reverse direction to remove a ball from the outtake
   * apparatus
   */
  public void outtakeBall() {
      intakeMotor.set(ControlMode.PercentOutput, Parameters.OUTTAKE_MOTOR_SPEED);
  }

  /**
   * Spins the bottom belt on the conveyor up. Ball will travel towards the
   * shooter
   */
  public void conveyorBeltUp() {
      conveyorMotor.set(ControlMode.PercentOutput, Parameters.CONVEYOR_MOTOR_SPEED);
  }

  public void conveyorBeltUpWithSensor() {
      if (conveyorSensor.get()) {
          return;
      } else{
          conveyorBeltUp();
      }
  }

  /**
   * Spins the bottom belt on the conveyor down. Ball will travel towards the
   * intake/ground
   */
  public void conveyorBeltDown() {
      conveyorMotor.set(ControlMode.PercentOutput, -Parameters.CONVEYOR_MOTOR_SPEED);
  }

  /**
   * Spins the shooter belt on the conveyor system so the ball shoots out of the
   * conveyor
   */
  public void spinConveyorShooter() {
      shooterMotor.set(ControlMode.PercentOutput, Parameters.SHOOTER_MOTOR_SPEED);
  }

  /**
   * Spins the shooter belt on the conveyor system backwards so the ball goes back
   * into the rest of the conveyor system
   */
  public void spinConveyorShooterReverse() {
      shooterMotor.set(ControlMode.PercentOutput, -Parameters.SHOOTER_MOTOR_SPEED);
  }

  /**
   * Intakes a ball, runs the ball through the conveyor system and out throught he
   * shooter
   */
  public void shootCurrentBalls() {
      spinConveyorShooter();
      conveyorBeltUp();
  }

  /**
   * Spins all motors in the intake and conveyor system in reverse to clear all
   * balls back onto the field. Used if there is a jam or too many balls are in
   * the conveyor
   */
  public void runConveyorSystemReverse() {
      outtakeBall();
      conveyorBeltDown();
      spinConveyorShooterReverse();
  }

  public boolean hasBall() {
      return conveyorSensor.get();
  } 

  public void stop() {
    conveyorMotor.set(ControlMode.PercentOutput, 0);
    intakeMotor.set(ControlMode.PercentOutput, 0);
    shooterMotor.set(ControlMode.PercentOutput, 0);
  }
}
