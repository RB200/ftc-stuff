package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;


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

    public OpticalFlowSensor(I2cDeviceSynch deviceClient){
        super(deviceClient,true);
        I2cAddr addr = I2cAddr.create7bit(0x28);
        this.deviceClient.setI2cAddress(addr);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage(); // communication over i2c
    }
    protected int readShort(Register reg){
        deviceClient.write8(0xF0,0x02);
        
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
    public int getDeltaX(){
        int low = readShort(Register.DELTA_X_L);
        int high = readShort(Register.DELTA_X_H) << 8;
        return high | low;
    }
    public int getDeltaY(){
        int low = readShort(Register.DELTA_Y_L);
        int high = readShort(Register.DELTA_Y_H) << 8;
        return high | low;

    }


}
