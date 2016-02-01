package edu.berkeley.cs.sdb.buildingmodel;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Bitmap;
import android.net.Uri;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.provider.MediaStore;
import android.util.Log;
import android.widget.ImageView;

import org.apache.http.HttpResponse;
import org.apache.http.HttpStatus;
import org.apache.http.StatusLine;
import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.entity.mime.MultipartEntityBuilder;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.protocol.BasicHttpContext;
import org.apache.http.protocol.HttpContext;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

;

public class ApplianceControl extends Activity {

    private static final String LOG_TAG = "SDB3D";
    private static final int CAPTURE_IMAGE_REQUEST_CODE = 410;
    private static final String IMAGE_UPLOAD_URL = "http://castle.cs.berkeley.edu:50012/";
    private static final String IMAGE_DIRECTORY = "/storage/emulated/0/DCIM/CAMERA";

    HttpClient mHttpClient;
    ImageView mImageView;
    private Uri mImageUri;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        mHttpClient = new DefaultHttpClient();
        mImageView = (ImageView) findViewById(R.id.imageView);

        invokeCameraApp();
    }

    /*
     * Invoked once the user has taken a picture
     */
    @Override
    public void onActivityResult(int requestCode, int resultCode, Intent intent) {
        if (requestCode == CAPTURE_IMAGE_REQUEST_CODE) {
            if (resultCode == RESULT_OK) {
                new UploadImageTask(mHttpClient, new File(mImageUri.getPath())).execute();
                displayImage();
            } else if (resultCode == RESULT_CANCELED) {
                // If user cancelled operation, just return them to camera app
                invokeCameraApp();
            } else {
                throw new RuntimeException("Failed to capture appliance image");
            }
        }
    }

    /*
     * Invoke existing camera application to obtain image of appliance
     * Based on code from http://developer.android.com/guide/topics/media/camera.html#intents
     */
    private void invokeCameraApp() {
        Intent intent = new Intent(MediaStore.ACTION_IMAGE_CAPTURE);
        mImageUri = getImageFileUri();
        intent.putExtra(MediaStore.EXTRA_OUTPUT, mImageUri);
        startActivityForResult(intent, CAPTURE_IMAGE_REQUEST_CODE);
    }

    /*
     * Display captured image on the screen and show control interface
     */
    private void displayImage() {
        try {
            Bitmap bitmap = MediaStore.Images.Media.getBitmap(this.getContentResolver(),
                    mImageUri);
//            Matrix m = new Matrix();
//            m.postRotate(90);
//            Bitmap bmp = Bitmap.createBitmap(bitmap, 0, 0, bitmap.getWidth(),
//                    bitmap.getHeight(), m, true);
            mImageView.setImageBitmap(bitmap);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }


    /*
     * Based on code from android.com/guide/topics/media/camera.html#saving-media
     */
    private Uri getImageFileUri() {
        if (!Environment.getExternalStorageState().equals(Environment.MEDIA_MOUNTED)) {
            throw new RuntimeException("No external storage mounted");
        }

        File mediaStorageDir = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_PICTURES), "ApplianceControl");
        if (!mediaStorageDir.exists()) {
            if (!mediaStorageDir.mkdirs()) {
                throw new RuntimeException("Failed to create directory " + mediaStorageDir.getPath());
            }
        }

        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        File f = new File(mediaStorageDir.getPath() + File.separator + "IMG_" + timeStamp + ".jpg");
        return Uri.fromFile(f);
    }

    /*
     * Uploads captured image to a server and measures the associated delay
     */
    private class UploadImageTask extends AsyncTask<Void, Void, Void> {
        private final HttpClient httpClient;
        private long startTime;
        private long endTime;
        private File imageFile;

        public UploadImageTask(HttpClient httpClient, File imageFile) {
            this.httpClient = httpClient;
            this.imageFile = imageFile;
        }

        @Override
        protected Void doInBackground(Void... voids) {
            startTime = System.currentTimeMillis();

            HttpContext localContext = new BasicHttpContext();
            HttpPost httpPost = new HttpPost(IMAGE_UPLOAD_URL);
            MultipartEntityBuilder multipartEntityBuilder = MultipartEntityBuilder.create();
            multipartEntityBuilder.addBinaryBody("file", imageFile);
            httpPost.setEntity(multipartEntityBuilder.build());

            try {
                Log.d(LOG_TAG, "Starting HTTP Post");
                HttpResponse response = httpClient.execute(httpPost, localContext);
                Log.d(LOG_TAG, "Finished HTTP Post");
                StatusLine statusLine = response.getStatusLine();
                if (statusLine.getStatusCode() != HttpStatus.SC_OK) {
                    String msg = String.format("HTTP Error %d: %s", statusLine.getStatusCode(),
                            statusLine.getReasonPhrase());
                    throw new RuntimeException(msg);
                }
            } catch (IOException e) {
                throw new RuntimeException(e);
            }

            endTime = System.currentTimeMillis();

            return null;
        }

        @Override
        protected void onPostExecute(Void v) {
            String msg = String.format("Image %s posted in %d msec\n", imageFile.getName(), endTime - startTime);
            Log.i(LOG_TAG, msg);
        }
    }
}