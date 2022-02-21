package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RotationControl{

    private final Motor rotationMotor;
    double TareAngle = 0;
    private double lastLegalDirection = 1.0;

    //constructor
   public RotationControl(int deviceID, int analogEncoderID){
          rotationMotor = MotorFactory.createRotationMotor(deviceID, analogEncoderID);  
   }

   public void SetAngle(double angle){
        rotationMotor.setAngle(angle);
   }

   public void SetTareAngle(double inputAngle){
       TareAngle=inputAngle;
   }
   
   public double GetTareAngle(){  
    return TareAngle;
   }

   public void completeLoopUpdate(){  
    rotationMotor.completeLoopUpdate();
   }



   public double getCurrentAngle(){
       return rotationMotor.getCurrentAngle();
   }


   
   public double getEncoderVoltage() {
        return rotationMotor.getEncoderVoltage();
   }


   
   public Motor getRotationMotor() {
       return rotationMotor;
   }



   public void resetEncoder() {
        rotationMotor.resetEncoder();
   }
   

   public double getPositionFromIntegratedSensor(){
       return rotationMotor.getPositionFromIntegratedSensor();
   }
   
   public double getAbsoluteEncoder() {
       return rotationMotor.getPosition();
   }

}

