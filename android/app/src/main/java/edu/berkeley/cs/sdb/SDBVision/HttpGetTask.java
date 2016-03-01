package edu.berkeley.cs.sdb.SDBVision;

import android.os.AsyncTask;

import java.io.IOException;

import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.Response;

public class HttpGetTask extends AsyncTask<Void, Void, String> {
    private OkHttpClient mHttpClient;
    private String mUrl;
    private Listener mListener;

    public interface Listener {
        void onResponse(String response);
    }

    public HttpGetTask(OkHttpClient httpClient, String url, Listener listener) {
        mHttpClient = httpClient;
        mUrl = url;
        mListener = listener;
    }

    @Override
    protected String doInBackground(Void... voids) {
        Request request = new Request.Builder()
                .url(mUrl)
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

        return result;
    }

    @Override
    protected void onPostExecute(String response) {
        mListener.onResponse(response);
    }
}
