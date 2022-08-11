package com.robotics.sergei.balancercontroller;

import android.os.Bundle;
import android.view.View;
import android.view.Menu;
import android.view.MenuItem;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.widget.EditText;
import android.widget.RelativeLayout;

// http://www.vogella.com/code/ApiDemos/src/com/example/android/apis/os/RotationVectorDemo.html

public class RotationSensorActivity extends RobotCommandActivity
    implements SensorEventListener
{
    private EditText sensorText;
    private SensorManager mSensorManager;
    private Sensor mAccelerometer;
    private View reticleView;
    private View centerBubbleView;
    private RelativeLayout root;

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_rotation_sensor);

        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER); //.TYPE_ROTATION_VECTOR);

        onCreateProcess();	// onCreate Process by RobotCommandActivity

        sensorText=(EditText) findViewById(R.id.sensorText);
        centerBubbleView=(View) findViewById(R.id.centerBubble);
        reticleView=(View) findViewById(R.id.reticle);
        root = (RelativeLayout) findViewById(R.id.rotationSensorLayout);
    }

    @Override
    protected void onResume() {
        System.out.println("RotationSensorActivity onResume");
        super.onResume();
        mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
    }

    @Override
    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(this);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    private long millisLast = 0L;
    private long millisLastSent = 0L;
    private float reticleRadius = 0.0f;
    private float screenCenterX = 0.0f;
    private float screenCenterY = 0.0f;
    private float bubbleFactorX = 40.0f;    // ratio between screenX and sensorX
    private float bubbleFactorY = 70.0f;

//    @Override
//    public void onLoad()
//    {
//
//    }

    @Override
    public void onSensorChanged(SensorEvent event) {

        if(millisLast == 0L)
        {
            reticleRadius = reticleView.getLayoutParams().width / 2.0f;

            screenCenterX = root.getWidth() / 2.0f - reticleRadius;
            screenCenterY = root.getHeight() / 2.0f - reticleRadius;

            reticleView.setX(screenCenterX);
            reticleView.setY(screenCenterY);

            screenCenterX = (root.getWidth() - centerBubbleView.getWidth()) / 2.0f;
            screenCenterY = (root.getHeight() - centerBubbleView.getHeight()) / 2.0f;
        }

        long millis = System.currentTimeMillis();

        if(millis - millisLast > 20)
        {
            millisLast = millis;

            // values in range: -9.3..9.3
            float sensorX = -event.values[0];
            float sensorY = event.values[1];

            //sensorText.setText("" + sensorX + "      " + sensorY + "    " + reticleRadius);
            //sensorText.setText("X: " + sensorX + " Y: " + sensorY);

            // bubble deflection from center:
            float deltaX = sensorX * bubbleFactorX;
            float deltaY = sensorY * bubbleFactorY;

            float bubbleX = screenCenterX + deltaX;
            float bubbleY = screenCenterY + deltaY;

            // move the bubble on screen:
            centerBubbleView.setX(bubbleX);
            centerBubbleView.setY(bubbleY);

            if(millis - millisLastSent > 100) {
                millisLastSent = millis;

                // bubble vector and its part outside reticle:
                double vectorLength = Math.min(450f, Math.sqrt(deltaX * deltaX + deltaY * deltaY));
                double outsideReticle = Math.min(250f, Math.max(vectorLength - reticleRadius, -0.1f));

                if (!this.mConnected) {
                    sensorText.setText(" Length: " + java.lang.Math.round(vectorLength) + "   Outside: " + java.lang.Math.round(outsideReticle));
                } else {

                    if ( outsideReticle < 0.0f ) {
                        //serialSend("s");                //send the data to the BLUNO
                        sensorText.setText("Stop");
                        DriveStopCommand();
                    } else {
                        float sensorXDeadZone = 2.0f;
                        float sensorYDeadZone = 2.0f;

                        // factors to cover the range -100...100:
                        float sensorFactorTurn = 100.0f / (9.3f - sensorXDeadZone);
                        float sensorFactorSpeed = -100.0f / (9.3f - sensorYDeadZone);

                        if(Math.abs(sensorX) > sensorXDeadZone)
                            sensorX = sensorX > 0 ? sensorX - sensorXDeadZone : sensorX + sensorXDeadZone;
                        else
                            sensorX = 0.0f;

                        if(Math.abs(sensorY) > sensorYDeadZone)
                            sensorY = sensorY > 0 ? sensorY - sensorYDeadZone : sensorY + sensorYDeadZone;
                        else
                            sensorY = 0.0f;

                        int turn = (int) Math.min(100.0, Math.max(-100.0, Math.round(sensorX * sensorFactorTurn)));
                        int speed = (int) Math.min(100.0, Math.max(-100.0, Math.round(sensorY * sensorFactorSpeed)));

                        sensorText.setText(" Turn: " + turn + "  Speed: " + speed);

                        DriveMoveCommand(turn, speed);
                    }
                }
            }
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.rotation_sensor, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();
        if (id == R.id.action_settings) {
            return true;
        }
        return super.onOptionsItemSelected(item);
    }

    @Override
    public void onSerialReceived(String theString) {							//Once connection data received, this function will be called
        // TODO Auto-generated method stub
        //serialReceivedText.append(theString);							//append the text into the EditText
        //The Serial data from the BLUNO may be sub-packaged, so using a buffer to hold the String is a good choice.

    }
}
