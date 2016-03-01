package edu.berkeley.cs.sdb.SDBVision;

import android.os.AsyncTask;

import org.apache.http.HttpResponse;
import org.apache.http.HttpStatus;
import org.apache.http.StatusLine;
import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.protocol.BasicHttpContext;
import org.apache.http.protocol.HttpContext;
import org.apache.http.util.EntityUtils;

import java.io.IOException;

public class HttpGetTask extends AsyncTask<Void, Void, String> {
    private HttpClient httpClient;
    private String url;
    private Listener mListener;

    public interface Listener {
        void onResponse(String response);
    }

    public HttpGetTask(HttpClient httpClient, String url, Listener listener) {
        this.httpClient = httpClient;
        this.url = url;
        this.mListener = listener;
    }

    @Override
    protected String doInBackground(Void... voids) {
        HttpContext localContext = new BasicHttpContext();
        HttpGet httpGet = new HttpGet(this.url);

        String result = null;
        try {
            HttpResponse response = httpClient.execute(httpGet, localContext);
            StatusLine statusLine = response.getStatusLine();
            if (statusLine.getStatusCode() != HttpStatus.SC_OK) {
                String msg = String.format("HTTP Error %d: %s", statusLine.getStatusCode(), statusLine.getReasonPhrase());
                throw new RuntimeException(msg);
            }
            result = EntityUtils.toString(response.getEntity());
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
