// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
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
import frc.robot.Parameters;
public class Climber extends SubsystemBase {
  
  DoubleSolenoid ArmSolenoid;
  DoubleSolenoid smallHookSolenoid;
  WPI_TalonFX leftMotor;
  WPI_TalonFX rightMotor;
  /** Creates a new Climber. */
  public Climber() {
    ArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Parameters.INTAKE_FORWARD_CHANNEL, Parameters.INTAKE_REVERSE_CHANNEL);
    smallHookSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Parameters.INTAKE_FORWARD_CHANNEL, Parameters.INTAKE_REVERSE_CHANNEL);
    leftMotor = new WPI_TalonFX(Parameters.LEFT_CLIMBER_ID);
    rightMotor = new WPI_TalonFX(Parameters.RIGHT_CLIMBER_ID);
  }

  @Override
  public void periodic() {

  }
}
