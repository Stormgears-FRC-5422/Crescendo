package frc.robot.commands.climb;

import frc.robot.RobotState;
import frc.robot.commands.StormCommand;
import frc.robot.subsystems.Climber;

public class SimpleGotoDegrees extends StormCommand {
    RobotState state;
    Climber climber;
    Climber.Direction direction;
    double angle;
    boolean earlyFinish = false;
    boolean forceWhenNotHomed = false;

    public SimpleGotoDegrees(Climber c, double degrees, Climber.Direction direction){
        climber = c;
        this.angle = degrees;
        this.direction = direction;
        this.state = RobotState.getInstance();
        addRequirements(c);
    }

    public void setTarget(double degrees) {
        log("Resetting target angle to " + degrees);
        this.angle = degrees;
    }

    public void forceWhenNotHomed(boolean force) {
        log("Setting forceWhenNotHomed = " + force);
        forceWhenNotHomed = force;
    }

    @Override
    public void initialize() {
        super.initialize();

        if (state.climberHasBeenHomed() || forceWhenNotHomed) {
            climber.setTargetDegrees(angle);
            climber.setClimberState(direction == Climber.Direction.FORWARD ?
                Climber.ClimberState.MOVE_FORWARD : Climber.ClimberState.MOVE_REVERSE);
            earlyFinish = false;
        } else {
            earlyFinish = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (earlyFinish) {
            log("Finishing early - must home first!");
            return true;
        }

        boolean result = climber.reachedTargetDegrees();
        if (result) {
            this.log("Climber is at the desired position");
        }
        return result;
    }

    @Override
    public void end(boolean interrupted) {
        climber.setClimberState(Climber.ClimberState.IDLE_BRAKE);
        super.end(interrupted);
    }

}
