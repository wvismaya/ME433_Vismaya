package com.example.vismayawalawalkar.homework_15;

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.WindowManager;
import android.widget.TextView;
import android.widget.SeekBar;

import java.io.IOException;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static android.graphics.Color.rgb;

import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.ScrollView;
import android.widget.SeekBar;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.CdcAcmSerialDriver;
import com.hoho.android.usbserial.driver.ProbeTable;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class MainActivity extends Activity implements TextureView.SurfaceTextureListener {
    private Camera mCamera;
    private TextureView mTextureView;
    private SurfaceView mSurfaceView;
    private SurfaceHolder mSurfaceHolder;
    private Bitmap bmp = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
    private Canvas canvas = new Canvas(bmp);
    private Paint paint1 = new Paint();
    private TextView mTextView;

    static long prevtime = 0; // for FPS calculation
    static int progressChanged = 0;

    static double xspot = 0;
    static double yspot0 = 0;
    static double yspot1 = 0;
    static int error = 0;
    static int lasterror = 0;
    static double Kp = 0.5;
    static double Kd = 0.2;
    static int motorspeed = 0;

    SeekBar myControl;
    SeekBar myPWM;
    TextView myTextView;
    TextView SendUSB;

    Button button;
    TextView myTextView2;
    ScrollView myScrollView;
    TextView myTextView3;
    private UsbManager manager;
    private UsbSerialPort sPort;
    private final ExecutorService mExecutor = Executors.newSingleThreadExecutor();
    private SerialInputOutputManager mSerialIoManager;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON); // keeps the screen from turning off

        mTextView = (TextView) findViewById(R.id.cameraStatus);

        // see if the app has permission to use the camera
        //ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.CAMERA}, 1);
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            mSurfaceView = (SurfaceView) findViewById(R.id.surfaceview);
            mSurfaceHolder = mSurfaceView.getHolder();

            mTextureView = (TextureView) findViewById(R.id.textureview);
            mTextureView.setSurfaceTextureListener(this);

            // set the paintbrush for writing text on the image
            paint1.setColor(0xffff0000); // red
            paint1.setTextSize(24);

            mTextView.setText("started camera");
        } else {
            mTextView.setText("no camera permissions");
        }

        myControl = (SeekBar) findViewById(R.id.seek1);
        myPWM = (SeekBar) findViewById(R.id.seek2);

        myTextView = (TextView) findViewById(R.id.textView01);
        myTextView.setText("Enter whatever you Like!");

        SendUSB = (TextView) findViewById(R.id.usbsend);
        SendUSB.setText("Enter value to send!");

        myScrollView = (ScrollView) findViewById(R.id.ScrollView01);
        myTextView3 = (TextView) findViewById(R.id.frompic);
        button = (Button) findViewById(R.id.button1);

        //@Override
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                SendUSB.setText("value on click is "+myPWM.getProgress());
                String sendString = String.valueOf(0) + '\n';
                try {
                    sPort.write(sendString.getBytes(), 10); // 10 is the timeout
                } catch (IOException e) { }

                // TODO Auto-generated method stub
                finish();
                System.exit(0);
            }
        });

        manager = (UsbManager) getSystemService(Context.USB_SERVICE);

        setMyControlListener();
    }

    /*USB Functions*/
    private final SerialInputOutputManager.Listener mListener =
            new SerialInputOutputManager.Listener() {
                @Override
                public void onRunError(Exception e) {

                }

                @Override
                public void onNewData(final byte[] data) {
                    MainActivity.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            MainActivity.this.updateReceivedData(data);
                        }
                    });
                }
            };

    @Override
    protected void onPause(){
        super.onPause();
        stopIoManager();
        if(sPort != null){
            try{
                sPort.close();
            } catch (IOException e){ }
            sPort = null;
        }
        finish();
    }

    @Override
    protected void onResume() {
        super.onResume();

        ProbeTable customTable = new ProbeTable();
        customTable.addProduct(0x04D8,0x000A, CdcAcmSerialDriver.class);
        UsbSerialProber prober = new UsbSerialProber(customTable);

        final List<UsbSerialDriver> availableDrivers = prober.findAllDrivers(manager);

        if(availableDrivers.isEmpty()) {
            //check
            return;
        }

        UsbSerialDriver driver = availableDrivers.get(0);
        sPort = driver.getPorts().get(0);

        if (sPort == null){
            //check
        }else{
            final UsbManager usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
            UsbDeviceConnection connection = usbManager.openDevice(driver.getDevice());
            if (connection == null){
                //check
                PendingIntent pi = PendingIntent.getBroadcast(this, 0, new Intent("com.android.example.USB_PERMISSION"), 0);
                usbManager.requestPermission(driver.getDevice(), pi);
                return;
            }

            try {
                sPort.open(connection);
                sPort.setParameters(9600, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);

            }catch (IOException e) {
                //check
                try{
                    sPort.close();
                } catch (IOException e1) { }
                sPort = null;
                return;
            }
        }
        onDeviceStateChange();
    }

    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        mCamera = Camera.open();
        Camera.Parameters parameters = mCamera.getParameters();
        parameters.setPreviewSize(640, 480);
        parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY); // no autofocusing
        parameters.setAutoExposureLock(false); // keep the white balance constant
        mCamera.setParameters(parameters);
        mCamera.setDisplayOrientation(90); // rotate to portrait mode

        try {
            mCamera.setPreviewTexture(surface);
            mCamera.startPreview();
        } catch (IOException ioe) {
            // Something bad happened
        }
    }

    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        // Ignored, Camera does all the work for us
    }

    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        mCamera.stopPreview();
        mCamera.release();
        return true;
    }

    // the important function
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        // every time there is a new Camera preview frame
        mTextureView.getBitmap(bmp);

        final Canvas c = mSurfaceHolder.lockCanvas();
        xspot = 0;
        yspot0 = 0;
        yspot1 = 0;
        int n = 1;
        boolean startFlag = true;
        for(int ii = 200; ii <280; ii++) { /*bmp.getHeight()*/
            if (c != null) {
                int thresh = 0; // comparison value
                int[] pixels = new int[bmp.getWidth()]; // pixels[] is the RGBA data
                int startY = ii; //240 // which row in the bitmap to analyze to read
                bmp.getPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1);

                // in the row, see if there is more green than red
                thresh = progressChanged;
                for (int i = 0; i < bmp.getWidth(); i++) {
                    if (  (red(pixels[i]) - green(pixels[i]) > thresh ) && (blue(pixels[i]) < 220-0.2*thresh ) ) {
                        pixels[i] = rgb(125, 0, 125); // over write the pixel with pure green
                        xspot = xspot + i;
                        n++;
                        /*if(startFlag){yspot0 = ii; startFlag = false;}
                        else {yspot1 = ii;}*/
                    }
                }
                // update the row
                bmp.setPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1);
            }
        }

        // draw a circle at some position
        int pos = 50; /*(int)((yspot0+yspot1)/2)*/
        canvas.drawCircle((int)(xspot/n), 240, 5, paint1); // x position, y position, diameter, color
        /* PID */
        /*
        error = 240 - (int)(xspot/n);
        motorspeed = (int)(Kp*error) + (int)(Kd*(error-lasterror));
        lasterror = error;
        */

        //Send
        /*
        String sendString = String.valueOf((int)(xspot/n)) + '\n';
        try {
            sPort.write(sendString.getBytes(), 10); // 10 is the timeout
        } catch (IOException e) { }
        */

        if(  ((int)(xspot/n) > 240 - 75) && ((int)(xspot/n) < 240 + 75) ){
            //Send 1
            String sendString = String.valueOf(1) + '\n';
            try {
                sPort.write(sendString.getBytes(), 10); // 10 is the timeout
            } catch (IOException e) { }
        }
        else if(  ((int)(xspot/n) > 240 + 75 + 30) && ((int)(xspot/n) < 450) ){
            //Send 2
            String sendString = String.valueOf(2) + '\n';
            try {
                sPort.write(sendString.getBytes(), 10); // 10 is the timeout
            } catch (IOException e) { }
        }
        else if(  ((int)(xspot/n) > 450) ){
            //Send 22
            String sendString = String.valueOf(22) + '\n';
            try {
                sPort.write(sendString.getBytes(), 10); // 10 is the timeout
            } catch (IOException e) { }
        }
        else if(  ((int)(xspot/n) < 240 - 75 - 30) && ((int)(xspot/n) > 10)){
            //Send 3
            String sendString = String.valueOf(3) + '\n';
            try {
                sPort.write(sendString.getBytes(), 10); // 10 is the timeout
            } catch (IOException e) { }
        }
        else if(  ((int)(xspot/n) < 10) && ((int)(xspot/n) != 0) ){
            //Send 33
            String sendString = String.valueOf(33) + '\n';
            try {
                sPort.write(sendString.getBytes(), 10); // 10 is the timeout
            } catch (IOException e) { }
        }
        else if(  ((int)(xspot/n) == 0) ){
            //Send 4
            String sendString = String.valueOf(4) + '\n';
            try {
                sPort.write(sendString.getBytes(), 10); // 10 is the timeout
            } catch (IOException e) { }
        }
        else{
            //Send 44
            String sendString = String.valueOf(44) + '\n';
                try {
                    sPort.write(sendString.getBytes(), 10); // 10 is the timeout
                } catch (IOException e) { }
            }


        // Set a huge delay to reduce screen sampling and intermittent yet frequent case 4
        int jj;
        for(jj=0; jj<1000; jj++){}

        // write the pos as text
        canvas.drawText("pos = " + xspot, 10, 200, paint1);
        c.drawBitmap(bmp, 0, 0, null);
        mSurfaceHolder.unlockCanvasAndPost(c);

        // calculate the FPS to see how fast the code is running
        long nowtime = System.currentTimeMillis();
        long diff = nowtime - prevtime;
        mTextView.setText("FPS " + 1000 / diff);
        prevtime = nowtime;
    }



    private void stopIoManager(){
        if(mSerialIoManager != null) {
            mSerialIoManager.stop();
            mSerialIoManager = null;
        }
    }

    private void startIoManager() {
        if(sPort != null){
            mSerialIoManager = new SerialInputOutputManager(sPort, mListener);
            mExecutor.submit(mSerialIoManager);
        }
    }

    private void onDeviceStateChange(){
        stopIoManager();
        startIoManager();
    }

    private void updateReceivedData(byte[] data) {
        //do something with received data

        //for displaying:
        String rxString = null;
        try {
            rxString = new String(data, "UTF-8"); // put the data you got into a string
            myTextView3.append(rxString);
            myScrollView.fullScroll(View.FOCUS_DOWN);
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
        }
    }

    /* Custom functions */

    private void setMyControlListener() {
        myControl.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {

            //progressChanged = 0;

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged = 2*progress - 100;
                myTextView.setText("The value is: "+progress);
            }

            //@Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            //@Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        myPWM.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {

            //progressChanged = 0;

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged = 2*progress - 100;
                SendUSB.setText("The value is: "+progress);
            }

            //@Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            //@Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }
}