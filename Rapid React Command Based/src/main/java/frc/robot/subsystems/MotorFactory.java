package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorFactory {

    public static enum MotorType {
        TALON, NEO
    }

    public static MotorType tractionMotorType = MotorType.TALON;
    public static MotorType rotationMotorType = MotorType.TALON;

    public static Motor createTractionMotor(int deviceID) {
        if (tractionMotorType == MotorType.TALON) {
            
            return new TalonFXFalcon(deviceID, false);

        } 
        // else if (tractionMotorType == MotorType.NEO) { 
        //         return new SparkMaxNeo(deviceID, IdleMode.kBrake, false);

        // }
        return null;
        
 }
    public static Motor createRotationMotor(int deviceID, int analogEncoderID) {
        if (rotationMotorType == MotorType.TALON) {
            boolean isDevIDLegal = deviceID > 14;
            
           // SmartDashboard.putBoolean("isDevIDLegal", isDevIDLegal);
            return new TalonFXFalcon(deviceID, NeutralMode.Brake, true, analogEncoderID);

        } 
        // else if (tractionMotorType == MotorType.NEO) {
        //     return new SparkMaxNeo(deviceID, IdleMode.kCoast, false);
        // }
        return null;
    }

    




}