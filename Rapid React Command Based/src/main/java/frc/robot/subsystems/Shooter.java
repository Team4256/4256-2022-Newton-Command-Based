// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parameters;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Conveyor.RaiseIntake;

public class Shooter extends SubsystemBase {

  private final TalonFX shooterMotor;
  private final TalonSRX leftOuterShooterMotor;
  private final TalonSRX rightOuterShooterMotor;
  private static Shooter instance = null;
  private static Conveyor conveyor = Conveyor.getInstance();
  private static Boolean isShootingHigh = false;
  

  public Shooter() {
    shooterMotor = new TalonFX(Parameters.SHOOTER_MOTOR_ID);
    leftOuterShooterMotor = new TalonSRX(Parameters.LEFT_OUTER_SHOOTER_MOTOR_ID);
    rightOuterShooterMotor = new TalonSRX(Parameters.RIGHT_OUTER_SHOOTER_MOTOR_ID);
    
  }

  public static synchronized Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }


  public void spinConveyorShooter() {
    shooterMotor.set(ControlMode.PercentOutput, Parameters.SHOOTER_MOTOR_SPEED);
  }

  public void spinOuterShooterHigh() {
    leftOuterShooterMotor.set(ControlMode.PercentOutput, Parameters.OUTER_SHOOTER_MOTOR_HIGH_SPEED);
    rightOuterShooterMotor.set(ControlMode.PercentOutput, -Parameters.OUTER_SHOOTER_MOTOR_HIGH_SPEED);
    
  }


  public void spinOuterShooterLow() {
    leftOuterShooterMotor.set(ControlMode.PercentOutput, Parameters.OUTER_SHOOTER_MOTOR_LOW_SPEED);
    rightOuterShooterMotor.set(ControlMode.PercentOutput, -Parameters.OUTER_SHOOTER_MOTOR_LOW_SPEED);
  }

  public void shooterWithOverride() {
    //spinConveyorShooter();
    conveyor.conveyorBeltUp();
    //spinOuterShooterHigh();

  }

  public void reverseChute() {
    leftOuterShooterMotor.set(ControlMode.PercentOutput, -Parameters.OUTER_SHOOTER_MOTOR_HIGH_SPEED);
    rightOuterShooterMotor.set(ControlMode.PercentOutput, Parameters.OUTER_SHOOTER_MOTOR_HIGH_SPEED);
    conveyor.runConveyorSystemReverse();
  }
  /**
   * Spins the shooter belt on the conveyor system backwards so the ball goes back
   * into the rest of the conveyor system
   */
  public void spinConveyorShooterReverse() {
    shooterMotor.set(
      ControlMode.PercentOutput,
      -Parameters.SHOOTER_MOTOR_SPEED
    );
  }

  public Boolean isShootingHigh() {
    return isShootingHigh;
  }

  /**
   * Intakes a ball, runs the ball through the conveyor system and out throught he
   * shooter
   */
  public void shootCurrentBalls() {

    if (isShootingHigh) {
      spinConveyorShooter();
      spinOuterShooterHigh();
    } else {
      spinConveyorShooter();
      spinOuterShooterLow();
    }

    
  }

  /**
   * Spins all motors in the intake and conveyor system in reverse to clear all
   * balls back onto the field. Used if there is a jam or too many balls are in
   * the conveyor
   */
  public void runConveyorSystemReverse() {
    spinConveyorShooterReverse();
    reverseChute();
  }

  public void setShootHigh() {
    isShootingHigh = true;
  }

  public void setShootLow() {
    isShootingHigh = false;
  }


  public void stop() {
    shooterMotor.set(ControlMode.PercentOutput, 0);
    leftOuterShooterMotor.set(ControlMode.PercentOutput, 0);
    rightOuterShooterMotor.set(ControlMode.PercentOutput, 0);
    conveyor.stop();
  }
}
