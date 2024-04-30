package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

//j
@I2cDeviceType
@DeviceProperties(name = "Optical Flow Sensor", xmlTag = "PMW3901")

public class OpticalFlowSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }
    protected synchronized boolean doInitialize(){
        return true;
    }
    protected enum Register{
        PRODUCT_ID(0x00),
        REVISION_ID(0x01),
        MOTION(0x02),
        DELTA_X_L(0x03),
        DELTA_X_H(0x04),
        DELTA_Y_L(0x05),
        DELTA_Y_H(0x06),
        SQUAL(0x07),
        RAWDATA_SUM(0x08),
        MAXIMUM_RAWDATA(0x09),
        MINIMUM_RAWDATA(0x0A),
        SHUTTER_LOWER(0x0B),
        SHUTTER_UPPER(0x0C),
        OBSERVATION(0x15),
        MOTION_BURST(0x16),
        POWER_UP_RESET(0x3A),
        RAWDATA_GRAB(0x58),
        RAWDATA_GRAB_STATUS(0x59),
        INVERSE_PRODUCT_ID(0x5F),
        LAST(INVERSE_PRODUCT_ID.bVal);
        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }
    public String getDeviceName(){
        return "Mikroe-3743";
    }

    public OpticalFlowSensor(I2cDeviceSynch deviceClient) throws InterruptedException {
        super(deviceClient,true);
        I2cAddr addr = I2cAddr.create7bit(0x28);
        this.deviceClient.setI2cAddress(addr);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage(); // communication over i2c

        byte[] powerUp = {(byte)(0x3A | 0x80), 0x5A};
        deviceClient.write(0x01,powerUp);
        sleep(10);
        //byte[] optimizationRegisters = {0x7F,0x61,0x7F,0x40,0x7F,0x41,0x43,0x45,0x5B,0x5F,0x7B,0x7F,0x44,0x40,0x4E};
        //byte[] optimizationValues = {0x00,(byte)0xAD,0x03,0x00,0x05,(byte)0xB3,(byte)0xF1,0x14,0x32,0x14,0x32,0x34,0x08,0x06,0x1B, (byte) 0xBF,0x3F};
        byte[] arr = {
                0x7F, 0x00,
                0x61, (byte) 0xAD,
                0x7F, 0x03,
                0x40, 0x00,
                0x7F, 0x05,
                0x41, (byte) 0xB3,
                0x43, (byte) 0xF1,
                0x45, 0x14,
                0x5B, 0x32,
                0x5F, 0x34,
                0x7B, 0x08,
                0x7F, 0x06,
                0x44, 0x1B,
                0x40, (byte) 0xBF,
                0x4E, 0x3F,
                0x7F, 0x08,
                0x65, 0x20,
                0x6A, 0x18,
                0x7F, 0x09,
                0x4F, (byte) 0xAF,
                0x5F, 0x40,
                0x48, (byte) 0x80,
                0x49, (byte) 0x80,
                0x57, 0x77,
                0x60, 0x78,
                0x61, 0x78,
                0x62, 0x08,
                0x63, 0x50,
                0x7F, 0x0A,
                0x45, 0x60,
                0x7F, 0x00,
                0x4D, 0x11,
                0x55, (byte) 0x80,
                0x74, 0x1F,
                0x75, 0x1F,
                0x4A, 0x78,
                0x4B, 0x78,
                0x44, 0x08,
                0x45, 0x50,
                0x64, (byte) 0xFF,
                0x65, 0x1F,
                0x7F, 0x14,
                0x65, 0x67,
                0x66, 0x08,
                0x63, 0x70,
                0x7F, 0x15,
                0x48, 0x48,
                0x7F, 0x07,
                0x41, 0x0D,
                0x43, 0x14,
                0x4B, 0x0E,
                0x45, 0x0F,
                0x44, 0x42,
                0x4C, (byte) 0x80,
                0x7F, 0x10,
                0x5B, 0x02,
                0x7F, 0x07,
                0x40, 0x41,
                0x70, 0x00};
                //delayMicroseconds(10000);
        byte[] postDelayArr = {
                    0x32, 0x44,
                    0x7F, 0x07,
                    0x40, 0x40,
                    0x7F, 0x06,
                    0x62, (byte)0xF0,
                    0x63, 0x00,
                    0x7F, 0x0D,
                    0x48, (byte)0xC0,
                    0x6F, (byte)0xD5,
                    0x7F, 0x00,
                    0x5B, (byte)0xA0,
                    0x4E, (byte)0xA8,
                    0x5A, 0x50,
                    0x40, (byte)0x80,
                    0x7F, 0x00,
                    0x5A, 0x10,
                    0x54, 0x00};

        for(int i = 0; i < arr.length-1; i+=2){
            //byte[] arr = {(byte)(optimizationRegisters[i] | 0x80),optimizationValues[i]};
            byte[] arr2 = {arr[i],arr[i+1]};
            deviceClient.write(0x01, arr2);
            deviceClient.read(0x01,2);
        }
        sleep(100);
        for(int i = 0; i < postDelayArr.length-1; i+=2){
            byte[] arr3 = {arr[i],arr[i+1]};
            deviceClient.write(0x01,arr3);
            deviceClient.read(0x01,2);
        }


        deviceClient.write8(0xF0,0x0C);


    }
    protected int readShort(Register reg){

        byte[] arr = {0x00,0x00};
        deviceClient.write(0x01,arr);
        //return TypeConversion.byteArrayToInt(deviceClient.read(0x00,2)); // should be 0x49
        byte[] response = deviceClient.read(0x01,2);
        //return response[0] + (response[1] << 8);
        return response[1];
    }

    public int getProductID(){
        return readShort(Register.PRODUCT_ID);
    }
    public byte[] readStuff(){

        byte[] arr = {0x02,0x02,0x03,0x03,0x04,0x04,0x05,0x05,0x06,0x06};
        deviceClient.write(0x01,arr);

        return deviceClient.read(0x01,10);
    }
    public short getDeltaX(){



        byte[] arr = {0x04,0x03,0x00};
        deviceClient.write(0x01,arr);

        byte[] response = deviceClient.read(0x01,3);

        byte[] conv = {response[1],response[2]};
        return TypeConversion.byteArrayToShort(conv);

    }
    public short getDeltaY(){

        byte[] arr = {0x06,0x05,0x00};
        deviceClient.write(0x01,arr);

        byte[] response = deviceClient.read(0x01,3);

        byte[] conv = {response[1],response[2]};
        return TypeConversion.byteArrayToShort(conv);
        //return response[2] * 256 + response[1];

    }


}
