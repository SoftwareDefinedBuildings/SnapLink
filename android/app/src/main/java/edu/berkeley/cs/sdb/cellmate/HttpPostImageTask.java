package edu.berkeley.cs.sdb.cellmate;

import android.os.AsyncTask;
import android.util.Log;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.text.SimpleDateFormat;
import java.util.Date;

import okhttp3.MediaType;
import okhttp3.MultipartBody;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.RequestBody;
import okhttp3.Response;

public class HttpPostImageTask extends AsyncTask<Void, Void, String> {
    private static final String LOG_TAG = "SDBVision";

    private static final MediaType MEDIA_TYPE_BINARY = MediaType.parse("application/octet-stream");

    private OkHttpClient mHttpClient;
    private String mUrl;
    private byte[] mImageData;
    private int mWidth;
    private int mHeight;
    private Listener mListener;

    public interface Listener {
        void onResponse(String response);
    }

    public HttpPostImageTask(OkHttpClient httpClient, String url, byte[] imageData, int width, int height, Listener listener) {
        mHttpClient = httpClient;
        mUrl = url;
        mImageData = imageData;
        mWidth = width;
        mHeight = height;
        mListener = listener;
    }

    @Override
    protected String doInBackground(Void... voids) {
        // Make sure the byte order is network order (big endian)
        byte[] widthBytes = ByteBuffer.allocate(4).order(ByteOrder.BIG_ENDIAN).putInt(mWidth).array();
        byte[] heightBytes = ByteBuffer.allocate(4).order(ByteOrder.BIG_ENDIAN).putInt(mHeight).array();

        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        RequestBody requestBody = new MultipartBody.Builder()
                .setType(MultipartBody.FORM)
                .addFormDataPart("file", timeStamp, RequestBody.create(MEDIA_TYPE_BINARY, mImageData))
                .addFormDataPart("width", timeStamp, RequestBody.create(MEDIA_TYPE_BINARY, widthBytes))
                .addFormDataPart("height", timeStamp, RequestBody.create(MEDIA_TYPE_BINARY, heightBytes))
                .build();
        Request request = new Request.Builder()
                .url(mUrl)
                .post(requestBody)
                .build();

        String result = null;
        try {
            Response response = mHttpClient.newCall(request).execute();
            if (response.isSuccessful()) {
                result = response.body().string();
            } else {
                Log.e(LOG_TAG, "HTTP Error " + response.code() + ":" + response.message());
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        if (result != null && !result.trim().equals("")) {
            return result.trim();
        }

        return null;
    }

    @Override
    protected void onPostExecute(String response) {
        mListener.onResponse(response);
    }
}
