package org.firstinspires.ftc.teamcode.util.robotAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.GameCircumstance;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometryForYuntai;

public class RingLauncher implements Runnable{
    DcMotor motorLaunchI, motorLaunchII;
    Servo servoAngle, servoPush;
    ThreeWheelOdometryForYuntai odometry;
    Position towelGoalPosition;

    public RingLauncher(GameCircumstance gameCircumstance, ThreeWheelOdometryForYuntai odometry){
        if(gameCircumstance.equals(GameCircumstance.AllianceColor.BLUE)){
            towelGoalPosition = new Position(35, 140.5);
        }else{
            towelGoalPosition = new Position(4 * 23, 140.5);
        }
        this.odometry = odometry;
    }
    private double getAimingAngle(){
        double absoluteAimingAngle = odometry.getCurrentPosition().getRelativePosition(towelGoalPosition).getAbsoluteAngle() - 90;
        double relativeAimingAngle = absoluteAimingAngle - odometry.angle();

        return Range.clip(relativeAimingAngle,-45,45);

    }

    private void aiming(){

    }

    @Override
    public void run() {

    }
}
