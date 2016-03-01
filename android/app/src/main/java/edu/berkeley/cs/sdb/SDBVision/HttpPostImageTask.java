package edu.berkeley.cs.sdb.SDBVision;

import android.media.Image;
import android.os.AsyncTask;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.text.SimpleDateFormat;
import java.util.Date;

import okhttp3.MediaType;
import okhttp3.MultipartBody;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.RequestBody;
import okhttp3.Response;

public class HttpPostImageTask extends AsyncTask<Void, Void, String> {
    private static final MediaType MEDIA_TYPE_JPEG = MediaType.parse("image/jpeg");

    private OkHttpClient mHttpClient;
    private String mUrl;
    private Image mImage;
    private Listener mListener;

    public interface Listener {
        void onResponse(String response);
    }

    public HttpPostImageTask(OkHttpClient httpClient, String url, Image image, Listener listener) {
        mHttpClient = httpClient;
        mUrl = url;
        mImage = image;
        mListener = listener;
    }

    @Override
    protected String doInBackground(Void... voids) {
        ByteBuffer buffer = mImage.getPlanes()[0].getBuffer();
        byte[] bytes = new byte[buffer.remaining()];
        buffer.get(bytes); // write buffer to bytes
        mImage.close();
        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        RequestBody requestBody = new MultipartBody.Builder()
                .setType(MultipartBody.FORM)
                .addFormDataPart("file", timeStamp + ".jpg", RequestBody.create(MEDIA_TYPE_JPEG, bytes))
                .build();
        Request request = new Request.Builder()
                .url(mUrl)
                .post(requestBody)
                .build();

        String result = null;
        try {
            Response response = mHttpClient.newCall(request).execute();
            if (!response.isSuccessful()) {
                String msg = String.format("HTTP Error %d: %s", response.body(), response.message());
                throw new RuntimeException(msg);
            }
            result = response.body().string();
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
