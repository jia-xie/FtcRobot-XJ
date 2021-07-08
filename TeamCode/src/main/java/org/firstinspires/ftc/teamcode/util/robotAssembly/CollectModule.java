package org.firstinspires.ftc.teamcode.util.robotAssembly;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.util.robotAssembly.CollectModule.TransferStatus.ANTI_TRANSFER;
import static org.firstinspires.ftc.teamcode.util.robotAssembly.CollectModule.TransferStatus.RESET;
import static org.firstinspires.ftc.teamcode.util.robotAssembly.CollectModule.TransferStatus.TRANSFER;
import static org.firstinspires.ftc.teamcode.util.robotAssembly.CollectModule.UpDownStatus.UP;

public class CollectModule implements Runnable {

    DcMotor motorTimingBelt, motorCollect;
    Servo servoUpDown;
    Gamepad gamepadI, gamepadII;
    NormalizedColorSensor colorSensorHalfWay, colorSensorEndCollect;
    int ringTakeInCounter = 0;
    //Index
    double upIndex = IndexData.upIndex;
    double downIndex = IndexData.downIndex;
    double takeInPowerIndex = IndexData.takeInPowerIndex;
    double ringDetectRedThreshold = 100;
    boolean isPreviouslyDetectedRing = false;
    boolean isRunning;
    enum TransferStatus{
        TRANSFER, ANTI_TRANSFER, RESET;
    }
    enum UpDownStatus{
        UP, DOWN;
    }

    /**
     * Construct a Collect Module to function fully in a subStream
     * @param opMode
     * @param motorTimingBelt
     * @param motorCollect
     * @param servoUpDown
     * @param colorSensorFront
     * @param colorSensorEnd
     */
    public CollectModule(OpMode opMode, DcMotor motorTimingBelt, DcMotor motorCollect,
                         Servo servoUpDown,
                         NormalizedColorSensor colorSensorFront, NormalizedColorSensor colorSensorEnd){
        this.motorTimingBelt = motorTimingBelt;
        this.motorCollect = motorCollect;
        this.servoUpDown = servoUpDown;
        gamepadI = opMode.gamepad1;
        gamepadII = opMode.gamepad2;
        this.colorSensorHalfWay = colorSensorFront;
        this.colorSensorEndCollect = colorSensorEnd;
    }

    //Action Functions

    private void transferRings(TransferStatus transferStatus){
        switch (transferStatus) {
            case RESET:
                motorTimingBelt.setPower(0);
                break;
            case TRANSFER:
                motorTimingBelt.setPower(takeInPowerIndex);
                break;
            case ANTI_TRANSFER:
                motorTimingBelt.setPower(-takeInPowerIndex);
                break;

        }
    }

    private void liftRings(UpDownStatus upDownStatus){
        switch (upDownStatus){
            case UP:
                servoUpDown.setPosition(upIndex);
                break;
            case DOWN:
                servoUpDown.setPosition(downIndex);
                break;
        }
    }

    //Detect Functions
    private NormalizedRGBA getFrontColor(){
        return  colorSensorHalfWay.getNormalizedColors();
    }
    private NormalizedRGBA getEndCollectColor(){
        return colorSensorEndCollect.getNormalizedColors();
    }
    private boolean isFrontDetectedRing(){
        if((getFrontColor().red) * 256 > ringDetectRedThreshold){
            return true;
        }else {
            return false;
        }
    }

    private boolean isThreeRingsInCollector(){
        if(getEndCollectColor().red * 256 > 100){
            return true;
        }else{
            return false;
        }
    }

    //Index Functions
    private int addRingCounter(){
        ringTakeInCounter++;
        return ringTakeInCounter;
    }

    public int resetRingCounter(){
        ringTakeInCounter = 0;
        return 0;
    }

    public int countRing(){
        boolean isCurrentlyDetectedRing = isFrontDetectedRing();
        if(isCurrentlyDetectedRing != isPreviouslyDetectedRing){
            if(isCurrentlyDetectedRing = true){
                addRingCounter();
            }
        }
        isPreviouslyDetectedRing = isCurrentlyDetectedRing;
        return ringTakeInCounter;
    }

    @Override
    public void run() {


        while(isRunning) {
            if(gamepadI.right_bumper) {
                transferRings(TRANSFER);
                if (countRing() > 3 && isThreeRingsInCollector()) {
                    liftRings(UP);
                }else{
                    liftRings(UpDownStatus.DOWN);
                }
            }else if(gamepadI.right_bumper){
                transferRings(ANTI_TRANSFER);
            }else{
                transferRings(RESET);
            }
        }
    }
}