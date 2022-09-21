package frc.robot.commands.UpperShooterCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShootHighWithDelay extends SequentialCommandGroup {
    SpinOuterShooter spinOuterShooter = SpinOuterShooter.getInstance();
    ShooterBeltUp shooterBeltUp =  ShooterBeltUp.getInstance();
    private static ShootHighWithDelay instance = null;

    public static synchronized ShootHighWithDelay getInstance() {
        if (instance == null) {
          instance = new ShootHighWithDelay();
        }
        return instance;
      }
    
    public ShootHighWithDelay () {
        addCommands(
            new ParallelDeadlineGroup(new SpinOuterShooter(), new WaitCommand(1)),
            new ParallelDeadlineGroup(shooterBeltUp, new SpinOuterShooter(), new WaitCommand(1))
        );
    }
}
