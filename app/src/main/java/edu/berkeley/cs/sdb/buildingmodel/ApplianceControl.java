package edu.berkeley.cs.sdb.buildingmodel;

import android.app.Activity;
import android.app.ProgressDialog;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.net.Uri;
import android.net.http.AndroidHttpClient;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.provider.MediaStore;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import org.apache.http.HttpResponse;
import org.apache.http.HttpStatus;
import org.apache.http.StatusLine;
import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.entity.FileEntity;
import org.apache.http.util.EntityUtils;

import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.ExecutionException;

public class ApplianceControl extends Activity {

    private static final int CAPTURE_IMAGE_REQUEST_CODE = 410;
    private static final String IMAGE_UPLOAD_URL = "http://castle.cs.berkeley.edu:55000/";
    private Uri mImageUri;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_applicance_control);
        setInterfaceVisiblity(View.INVISIBLE);
        invokeCameraApp();
    }

    // Invoked once the user has taken a picture
    @Override
    public void onActivityResult(int requestCode, int resultCode, Intent intent) {
        if (requestCode == CAPTURE_IMAGE_REQUEST_CODE) {
            if (resultCode == RESULT_OK) {
//                try {
//                    String result = new UploadImageTask().execute().get();
//                    Log.d("ApplianceControl", result);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                } catch (ExecutionException e) {
//                    throw new RuntimeException(e);
//                }
                displayImage();
            } else if (resultCode == RESULT_CANCELED) {
                // If user cancelled operation, just return them to camera app
                invokeCameraApp();
            } else {
                throw new RuntimeException("Failed to capture appliance image");
            }
        }
    }

    // Display captured image on the screen and show control interface
    private void displayImage() {
        try {
            ImageView imageView = (ImageView) findViewById(R.id.imageView);
            Bitmap bitmap = MediaStore.Images.Media.getBitmap(this.getContentResolver(),
                    mImageUri);
//            Matrix m = new Matrix();
//            m.postRotate(90);
//            Bitmap bmp = Bitmap.createBitmap(bitmap, 0, 0, bitmap.getWidth(),
//                    bitmap.getHeight(), m, true);
            imageView.setImageBitmap(bitmap);
            setInterfaceVisiblity(View.VISIBLE);
        } catch (IOException e) {
            throw new RuntimeException(e);
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


    // Based on code from android.com/guide/topics/media/camera.html#saving-media
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

    private void setInterfaceVisiblity(int visibility) {
        Button onButton = (Button) findViewById(R.id.on_button);
        onButton.setVisibility(visibility);
        Button offButton = (Button) findViewById(R.id.off_button);
        offButton.setVisibility(visibility);
        Button dimUpButton = (Button) findViewById(R.id.dim_up_button);
        dimUpButton.setVisibility(visibility);
        Button dimDownButton = (Button) findViewById(R.id.dim_down_button);
        dimDownButton.setVisibility(visibility);
        TextView dimText = (TextView) findViewById(R.id.dim_label);
        dimText.setVisibility(visibility);
    }

    // Uploads captured image to a server and measures the associated delay
    private class UploadImageTask extends AsyncTask<Void, Void, String> {
        private ProgressDialog progress;
        private long startTime;
        private long endTime;

        @Override
        protected void onPreExecute() {
            progress = new ProgressDialog(ApplianceControl.this);
            progress.setTitle("Loading");
            progress.setMessage("Uploading image to server...");
            progress.show();
        }

        @Override
        protected String doInBackground(Void... voids) {
            HttpClient client = AndroidHttpClient.newInstance("ApplianceControl");
            HttpPost httpPost = new HttpPost(IMAGE_UPLOAD_URL);

            try {
                httpPost.setEntity(new FileEntity(new File(new URI(mImageUri.toString())), "image/jpeg"));
                startTime = System.currentTimeMillis();
                HttpResponse response = client.execute(httpPost);
                endTime = System.currentTimeMillis();
                StatusLine statusLine = response.getStatusLine();
                if (statusLine.getStatusCode() != HttpStatus.SC_OK) {
                    String msg = String.format("HTTP Error %d: %s", statusLine.getStatusCode(),
                            statusLine.getReasonPhrase());
                    throw new RuntimeException(msg);
                }

                return EntityUtils.toString(response.getEntity(), "UTF-8");

            } catch (IOException e) {
                throw new RuntimeException(e);
            } catch (URISyntaxException e) {
                throw new RuntimeException(e);
            } finally {
                client.getConnectionManager().shutdown();
            }
        }

        @Override
        protected void onPostExecute(String result) {
            progress.dismiss();
            String msg = String.format("Image posting delay: %dms", endTime - startTime);
            Toast.makeText(ApplianceControl.this, msg, Toast.LENGTH_LONG).show();
        }
    }
}