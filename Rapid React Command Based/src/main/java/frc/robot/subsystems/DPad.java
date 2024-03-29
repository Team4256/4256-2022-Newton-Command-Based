package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.Button;

public class DPad extends Button {

    Xbox controller;
    Direction direction;

    public DPad(Xbox controller, Direction direction) {
        this.controller = controller;
        this.direction = direction;
    }

    public static enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        int direction;

        private Direction(int direction) {
            this.direction = direction;
        }
    }

    public boolean get() {
        int dPadValue = controller.getPOV();
        return (dPadValue == direction.direction) || (dPadValue == (direction.direction + 45) % 360)
                || (dPadValue == (direction.direction + 315) % 360);
    }

}