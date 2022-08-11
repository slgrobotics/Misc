package com.robotics.sergei.balancercontroller;

import android.content.Intent;
import android.os.PowerManager;
import android.content.Context;
import android.view.WindowManager;
import android.widget.Button;

import com.trackroamer.arduinocomm.ToArduino;

/**
 * Created by sergei on 7/27/2014.
 * a base class for any activity that uses BLINO library and has a "Scan" button.
 * Takes care of common commands based on ToArduino class
 * Prints out data received from Arduino
 */
public class RobotCommandActivity extends BlunoLibrary
{
    protected Button buttonScan;
    protected PowerManager.WakeLock mWakeLock;

    @Override
    public void onCreateProcess()
    {
        super.onCreateProcess();												// onCreate Process by BlunoLibrary

        serialBegin(115200);													// set the Uart Baudrate on BLE chip to 115200

        buttonScan = findViewById(R.id.buttonScan);					// initial the button for scanning the BLE device
        buttonScan.setOnClickListener(v -> {
            buttonScanOnClickProcess();										// Alert Dialog for selecting the BLE device
        });

        //getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        final PowerManager pm = (PowerManager) getSystemService(Context.POWER_SERVICE);
        this.mWakeLock = pm.newWakeLock(PowerManager.SCREEN_DIM_WAKE_LOCK, "BalancerControllerTag");
        this.mWakeLock.acquire();
    }

    @Override
    public void onSerialReceived(String theString) {							// Once connection data received, this function will be called
        System.out.println("RobotCommandActivity : onSerialReceived() : " + theString);
    }

    @Override
    protected void onResume(){
        super.onResume();
        System.out.println("RobotCommandActivity : onResume");
        onResumeProcess();														// onResume Process by BlunoLibrary
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        onActivityResultProcess(requestCode, resultCode, data);					// onActivityResult Process by BlunoLibrary
        super.onActivityResult(requestCode, resultCode, data);
    }

    @Override
    protected void onPause() {
        super.onPause();
        onPauseProcess();														// onPause Process by BlunoLibrary
    }

    @Override
    protected void onStop() {
        super.onStop();
        onStopProcess();														// onStop Process by BlunoLibrary
    }

    @Override
    protected void onDestroy() {
        this.mWakeLock.release();
        super.onDestroy();
        onDestroyProcess();														// onDestroy Process by BlunoLibrary
    }

    @Override
    public void onConectionStateChange(connectionStateEnum theConnectionState)  // Once connection state changes, this function will be called
    {
        switch (theConnectionState) {											// Four connection state
            case isConnected:
                buttonScan.setText("Connected");
                break;
            case isConnecting:
                buttonScan.setText("Connecting");
                break;
            case isToScan:
                buttonScan.setText("Scan");
                break;
            case isScanning:
                buttonScan.setText("Scanning");
                break;
            case isDisconnecting:
                buttonScan.setText("isDisconnecting");
                break;
            default:
                break;
        }
    }

    private final int CHANNEL_IDENTIFY  = 0;    // send back device ident string
    private final int CHANNEL_DRIVE     = 1;    // drive control

    private final int CMD_IDENTIFY      = 127;
    private final int CMD_SET_VALUES    = 1;    // any channel

    private final int CMD_DRIVE_STOP    = 10;
    private final int CMD_DRIVE_MOVE    = 11;

    protected void DriveStopCommand()
    {
        ToArduino cmd = new ToArduino();
        cmd.channel = CHANNEL_DRIVE;
        cmd.command = CMD_DRIVE_STOP;

        String toSend = cmd.toString();

        //System.out.println("DriveStopCommand: '" + toSend + "'");

        serialSend(toSend);
    }
    protected void DriveMoveCommand(int turn, int speed)
    {
        ToArduino cmd = new ToArduino();
        cmd.channel = CHANNEL_DRIVE;
        cmd.command = CMD_DRIVE_MOVE;
        cmd.commandValues = new int[] { turn, speed };

        String toSend = cmd.toString();

        //System.out.println("DriveMoveCommand: '" + toSend + "'");

        serialSend(toSend);
    }
}
