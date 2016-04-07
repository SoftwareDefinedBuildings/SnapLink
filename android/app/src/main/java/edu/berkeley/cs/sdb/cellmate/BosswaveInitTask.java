package edu.berkeley.cs.sdb.cellmate;

import android.os.AsyncTask;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.concurrent.Semaphore;

import edu.berkeley.cs.sdb.bosswave.BosswaveClient;
import edu.berkeley.cs.sdb.bosswave.Response;
import edu.berkeley.cs.sdb.bosswave.ResponseHandler;

public class BosswaveInitTask extends AsyncTask<Void, Void, Void> {
    private BosswaveClient mBosswaveClient;
    private byte[] mKey;
    private Listener mTaskListener;
    private Semaphore mSem;

    public interface Listener {
        void onResponse();
    }

    public BosswaveInitTask(BosswaveClient bosswaveClient, byte[] key, Listener listener) {
        mBosswaveClient = bosswaveClient;

        mKey = key;
        mTaskListener = listener;
        mSem = new Semaphore(0);
    }

    private ResponseHandler mResponseHandler = new ResponseHandler() {
        @Override
        public void onResponseReceived(Response response) {
            mSem.release();
        }
    };

    @Override
    protected Void doInBackground(Void... voids) {
        try {
            mBosswaveClient.connect();

            File tempKeyFile = File.createTempFile("key", null, null);
            tempKeyFile.deleteOnExit();
            FileOutputStream fos = new FileOutputStream(tempKeyFile);
            fos.write(mKey);
            mBosswaveClient.setEntityFile(tempKeyFile, mResponseHandler);
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            mSem.acquire();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return null;
    }

    @Override
    protected void onPostExecute(Void result) {
        mTaskListener.onResponse();
    }
}
