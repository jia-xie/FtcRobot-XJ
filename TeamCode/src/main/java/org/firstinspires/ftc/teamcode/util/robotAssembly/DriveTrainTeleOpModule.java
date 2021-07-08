package org.firstinspires.ftc.teamcode.util.robotAssembly;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.GameCircumstance;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.control.AngularPID;
import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometryForYuntai;

@Config
public class DriveTrainTeleOpModule implements Runnable{

    Gamepad gamepadI, gamepadII;
    HardwareMap hardwareMap;
    int speedMax;
    DriveTrainMotionManagement driveTrain = new DriveTrainMotionManagement();
    boolean isRunning = true;
    Position towerShotPosition;
    AngularPID angularPID = new AngularPID(0.04, 0, 0.003, 0, 1, 0, 1.5, 0);
    ThreeWheelOdometryForYuntai odometry;
    private double launchAngle = 0;
    double updateTimeInterval = 300;//in miiliseconds
    private boolean angleLockSwitch, aLast = false;
    ElapsedTime runtime = new ElapsedTime();
    private double speedIndex = 1;
    private double x, y, w = 0;
    public DriveTrainTeleOpModule(Gamepad gamepadI, Gamepad gamepadII,
                                  HardwareMap hardwareMap, int speedMax,
                                  GameCircumstance.AllianceColor allianceColor,
                                  ThreeWheelOdometryForYuntai odometry){
        this.gamepadI = gamepadI;
        this.gamepadII = gamepadII;
        this.hardwareMap = hardwareMap;
        this.speedMax = speedMax;
        if(allianceColor == GameCircumstance.AllianceColor.BLUE){
            towerShotPosition = new Position(35, 132.125);
        }else if(allianceColor == GameCircumstance.AllianceColor.RED){
            towerShotPosition = new Position(98.875, 132.125);
        }
        this.odometry = odometry;
    }

    private double getLaunchAngle(ThreeWheelOdometryForYuntai odometry){
        Position relativePosition = odometry.getCurrentPosition().setRelativeY(4).getRelativePosition(towerShotPosition);
        return relativePosition.getAbsoluteAngle() + launchAngle - 90;
    }



    private void gamepadUpdate(){
        x = gamepadI.left_stick_x;
        y = -gamepadI.left_stick_y;
        w = gamepadI.right_stick_x;

        if(gamepadI.dpad_left){
            w = -0.2;
        }else if(gamepadI.dpad_right){
            w = 0.2;
        }

        speedIndex = 1 - gamepadI.right_trigger * 0.9;
        x *= speedIndex;
        y *= speedIndex;
        w *= speedIndex;

        boolean aCurrent = gamepadI.a;
        if((aLast != aCurrent) && (aCurrent == false))angleLockSwitch = !angleLockSwitch;
        if(gamepadI.right_stick_x != 0f || gamepadII.right_stick_x != 0f  ||
                gamepadI.dpad_left || gamepadI.dpad_right) angleLockSwitch = false;
        aLast = aCurrent;
    }
    public boolean getAngleLockSwitch(){
        return angleLockSwitch;
    }
    public void stopThread(){
        isRunning = false;
    }



    @Override
    public void run() {
        driveTrain.initDriveTrain(hardwareMap,speedMax);
        while(isRunning){
            gamepadUpdate();

            if(angleLockSwitch){
                if(runtime.milliseconds()>updateTimeInterval){
                    angularPID.angularPIDUpdate(getLaunchAngle(odometry));
                    runtime.reset();
                }
                double angularVelocity = angularPID.getAngularPIDOut(odometry.angle());
                if(!gamepadI.atRest() || angleLockSwitch){
                    driveTrain.moveAbsolutely(gamepadI, angularVelocity, odometry);
                }else{
                    driveTrain.stop();
                }
            }else{
                if(!gamepadI.atRest()){
                    driveTrain.movePower(x, y, w);
                }else{
                    driveTrain.stop();
                }
            }
        }
    }
}
