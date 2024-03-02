package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;

import frc.robot.Constants;
import frc.robot.Constants.SparkMax;
import frc.utils.motorcontrol.LimitSwitch;


public class Climber {
    double m_climberMotorSpeed = 0;
    

    private final CANSparkMax climberLeadMotor;
    private final CANSparkMax climberFollowerMotor;

    private SparkLimitSwitch climberForwardLimitSwitch;
    private SparkLimitSwitch climberZeroLimitSwitch;



    public Climber(){
        climberLeadMotor = new CANSparkMax(Constants.Climber.leaderID, CANSparkLowLevel.MotorType.kBrushless);
        climberFollowerMotor = new CANSparkMax(Constants.Climber.followerID, CANSparkLowLevel.MotorType.kBrushless);
        

        climberLeadMotor.setInverted(true);
        climberFollowerMotor.follow(climberLeadMotor, true);
        


        climberForwardLimitSwitch = climberLeadMotor.getForwardLimitSwitch(Type.kNormallyClosed);
        climberZeroLimitSwitch = climberLeadMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    }
    public void Climb(){
        climberLeadMotor.set(Constants.Climber.speed);
        
    }
    public void Descend(){
        climberLeadMotor.set(-Constants.Climber.speed);
    }
    public boolean isTooFarForward(){
        return climberForwardLimitSwitch.isPressed();
    }
    public boolean isTooFarBack(){
        return climberZeroLimitSwitch.isPressed();
    }
    public void Stop(){
        climberLeadMotor.set(0);
    }
     

    




    
}
