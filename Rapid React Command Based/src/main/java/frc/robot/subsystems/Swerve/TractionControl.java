package frc.robot.subsystems.Swerve;

public class TractionControl{

    private final Motor tractionMotor;

    //constructor
   public TractionControl(int deviceID){
       tractionMotor = MotorFactory.createTractionMotor(deviceID);
      
    }

   public void set(double speed){
        tractionMotor.setSpeed(speed);
   }

   public void completeLoopUpdate(){
       tractionMotor.completeLoopUpdate();
   }
   

   public double getRPS(){
    return tractionMotor.getRPS();
   }

   public double getRPM() {
       return tractionMotor.getRPM();
   }


   public void resetEncoder() {
    tractionMotor.resetEncoder();
   }

   public double getPositionFromIntegratedSensor(){
   return tractionMotor.getPositionFromIntegratedSensor();
   }

}

