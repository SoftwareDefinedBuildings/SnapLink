package edu.berkeley.cs.sdb.buildingmodel;

import android.app.Activity;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.provider.MediaStore;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;

public class ApplicanceControl extends Activity {

    private static final int CAPTURE_IMAGE_REQUEST_CODE = 410;
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
                ImageView imageView = (ImageView) findViewById(R.id.imageView);
                imageView.setImageURI(mImageUri);
                setInterfaceVisiblity(View.VISIBLE);
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
        Button onButton = (Button)findViewById(R.id.on_button);
        onButton.setVisibility(visibility);
        Button offButton = (Button)findViewById(R.id.off_button);
        offButton.setVisibility(visibility);
        Button dimUpButton = (Button)findViewById(R.id.dim_up_button);
        dimUpButton.setVisibility(visibility);
        Button dimDownButton = (Button)findViewById(R.id.dim_down_button);
        dimDownButton.setVisibility(visibility);
        TextView dimText = (TextView)findViewById(R.id.dim_label);
        dimText.setVisibility(visibility);
    }
}
