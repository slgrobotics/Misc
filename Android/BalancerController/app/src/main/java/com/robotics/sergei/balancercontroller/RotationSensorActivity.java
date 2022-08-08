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
    private float bubbleFactorX = 50.0f;
    private float bubbleFactorY = 80.0f;

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
            float sensorX = -event.values[0];
            float sensorY = event.values[1];

            //sensorText.setText("" + sensorX + "      " + sensorY + "    " + reticleRadius);

            //sensorText.setText("" + sensorX + "      " + sensorY);

            float deltaX = sensorX * bubbleFactorX;
            float deltaY = sensorY * bubbleFactorY;

            float bubbleX = screenCenterX + deltaX;
            float bubbleY = screenCenterY + deltaY;

            centerBubbleView.setX(bubbleX);
            centerBubbleView.setY(bubbleY);

            if(millis - millisLastSent > 100) {
                millisLastSent = millis;

                double vectorLength = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                double outsideReticle = vectorLength - reticleRadius;

                sensorText.setText("" + vectorLength + "      " + outsideReticle);

                if (this.mConnected) {

                    if ( outsideReticle < 0.0 ) {
                        //serialSend("s");                //send the data to the BLUNO
                        DriveStopCommand();
                    } else {
                        float inputFactorTurn = 0.1f;
                        float inputFactorSpeed = 0.2f;

                        int turn = (int) (sensorX * inputFactorTurn * outsideReticle);
                        int speed = (int) (sensorY * inputFactorSpeed * outsideReticle);

                        sensorText.setText("" + turn + "      " + speed + "    " + ((int)reticleRadius));

                        DriveMoveCommand(turn, speed);
                    }

                    /*
                    } else if (inputY > reticleRadiusInt) {
                        serialSend("a");
                    } else if (inputY < -reticleRadiusInt) {
                        serialSend("b");
                    } else if (inputX > reticleRadiusInt) {
                        serialSend("r");
                    } else if (inputX < -reticleRadiusInt) {
                        serialSend("l");
                    }
                    */
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
