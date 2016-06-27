package com.example.mark.myapplication;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.drawable.BitmapDrawable;

import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;


import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.objdetect.CascadeClassifier;
import org.opencv.imgproc.Imgproc;

public class MainActivity extends AppCompatActivity {
    //static{
     //   System.loadLibrary("jniTest");
    //}
    private Button btn,btn1;
    private ImageView img;


    myNDK jnifunc = new myNDK();
    @Override
    protected void onCreate(Bundle savedInstanceState) {
      //  Log.i("MainActivity", "called onCreate");

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);


        initView();
    }

    private void initView() {
        btn= (Button) findViewById(R.id.btn);
        btn1= (Button) findViewById(R.id.btn1);
        img= (ImageView) findViewById(R.id.img);

        btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
               Bitmap bitmap = ((BitmapDrawable) getResources().getDrawable(
                      R.drawable.f00000 )).getBitmap();

                int w = bitmap.getWidth(), h = bitmap.getHeight();
                int[] pix = new int[w * h];

                bitmap.getPixels(pix, 0, w, 0, 0, w, h);
               // Log.d(   "step 0 =",Integer.toString( jnifunc.geCount()  ) );

                int [] resultPixes = jnifunc.gray(pix,w,h);

            //    Log.d(   "step 1=",Float.toString( jnifunc.geCount()  ) );
              //  jnifunc.setone();
               // Log.d(   "step 2 =",Integer.toString( jnifunc.geCount()  ) );

                Bitmap result = Bitmap.createBitmap(w,h, Bitmap.Config.RGB_565);
               result.setPixels(resultPixes, 0, w, 0, 0,w, h);
                img.setImageBitmap(result);
            }
        }
        );

        btn1.setOnClickListener(new View.OnClickListener() {
            @Override

            public void onClick(View v) {
           //     myNDK testNDK = new myNDK();
            //  Log.d("MainActivity", testNDK.getMystring());

                if(!OpenCVLoader.initDebug() ){
                 Log.d("MainActivity", "Log from Jni =");
                }else {
             //       Log.d("MainActivity", testNDK.getMystring());
                }
//testNDK.getMystring()
             //   }
              // Mat rgbMat =  new Mat();
               //Mat grayMat = new Mat();
               //Bitmap srcBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.ic);
               // Bitmap grayBitmap = Bitmap.createBitmap(srcBitmap.getWidth(), srcBitmap.getHeight(), Bitmap.Config.RGB_565);
               // Utils.bitmapToMat(srcBitmap, rgbMat);//convert original bitmap to Mat, R G B.
               // Imgproc.cvtColor(rgbMat, grayMat, Imgproc.COLOR_RGB2GRAY);//rgbMat to gray grayMat
               // Utils.matToBitmap(grayMat, grayBitmap); //convert mat to bitmap
               // img.setImageBitmap(grayBitmap);
            }



        });
    }




}
