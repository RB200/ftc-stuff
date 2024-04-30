package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "Optical Flow Test")
public class OpticalFlowTest extends LinearOpMode {
    private OpticalFlowSensor opticalFlow;

    public void runOpMode() throws InterruptedException {

        opticalFlow = hardwareMap.get(OpticalFlowSensor.class, "opt_flow");
        waitForStart();
        while(opModeIsActive()){
            //byte[] prodID = opticalFlow.getProductID();
            byte[] result = opticalFlow.readStuff();
            for(int i = 0; i < result.length; i++){
                telemetry.addData("byte" + i + ": ",result[i]);
            }

            //telemetry.addData("Inverse Product ID: ", result[4]);
            //telemetry.addData("Delta X: ", opticalFlow.getDeltaX());
            //telemetry.addData("Delta Y: ", opticalFlow.getDeltaY());

            telemetry.update();
        }
    }
}
