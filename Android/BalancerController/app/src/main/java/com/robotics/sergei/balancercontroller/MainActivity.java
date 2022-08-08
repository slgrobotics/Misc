package com.robotics.sergei.balancercontroller;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import com.trackroamer.arduinocomm.ToArduino;

public class MainActivity  extends RobotCommandActivity
{
    private Button buttonSerialSend;
    private EditText serialSendText;
    private EditText serialReceivedText;

    private Button buttonStop;
    private Button buttonForward;
    private Button buttonBack;
    private Button buttonLeft;
    private Button buttonRight;
    private Button buttonByRotationSensor;

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        onCreateProcess();														// onCreate Process by RobotCommandActivity

        serialReceivedText=(EditText) findViewById(R.id.serialReceivedText);	// initial the EditText of the received data
        serialSendText=(EditText) findViewById(R.id.serialSendText);			// initial the EditText of the sending data

        buttonSerialSend = (Button) findViewById(R.id.buttonSerialSend);		// initial the button for sending the data
        buttonSerialSend.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                serialSend(serialSendText.getText().toString());				// send the data to the BLUNO
            }
        });

        buttonScan = (Button) findViewById(R.id.buttonScan);					// initial the button for scanning the BLE device
        buttonScan.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                buttonScanOnClickProcess();										// Alert Dialog for selecting the BLE device
            }
        });

        buttonStop = (Button) findViewById(R.id.buttonStop);
        buttonStop.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                //serialSend("s");				//send the data to the BLUNO
                DriveStopCommand();
            }
        });

        buttonForward = (Button) findViewById(R.id.buttonForward);
        buttonForward.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                //serialSend("a");				//send the data to the BLUNO
                DriveMoveCommand(0, 500);
            }
        });

        buttonBack = (Button) findViewById(R.id.buttonBack);
        buttonBack.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                //serialSend("b");				//send the data to the BLUNO
                DriveMoveCommand(0, -500);
            }
        });

        buttonLeft = (Button) findViewById(R.id.buttonLeft);
        buttonLeft.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                //serialSend("l");				//send the data to the BLUNO
                DriveMoveCommand(-300, 0);
            }
        });

        buttonRight = (Button) findViewById(R.id.buttonRight);
        buttonRight.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                //serialSend("r");				//send the data to the BLUNO
                DriveMoveCommand(300, 0);
            }
        });

        buttonByRotationSensor = (Button) findViewById(R.id.buttonByRotationSensor);
        buttonByRotationSensor.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                Intent intent = new Intent(v.getContext(), RotationSensorActivity.class);
                startActivity(intent);
            }
        });
    }

    @Override
    public void onSerialReceived(String theString) {							// Once connection data received, this function will be called
        serialReceivedText.setText(theString);
        super.onSerialReceived(theString);
    }
}
