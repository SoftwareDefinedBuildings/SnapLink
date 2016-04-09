package edu.berkeley.cs.sdb.cellmate;

import android.os.AsyncTask;

import java.io.File;
import java.io.IOException;
import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.berkeley.cs.sdb.bosswave.BosswaveClient;
import edu.berkeley.cs.sdb.bosswave.Response;
import edu.berkeley.cs.sdb.bosswave.ResponseHandler;

public class BosswaveInitTask extends AsyncTask<Void, Void, Boolean> {
    private BosswaveClient mBosswaveClient;
    private File mKeyFile;
    private Listener mTaskListener;
    private Semaphore mSem;
    private AtomicBoolean mSuccess;

    public interface Listener {
        void onResponse(boolean success);
    }

    public BosswaveInitTask(BosswaveClient bosswaveClient, File keyFile, Listener listener) {
        mBosswaveClient = bosswaveClient;
        mKeyFile = keyFile;
        mTaskListener = listener;
        mSem = new Semaphore(0);
        mSuccess = new AtomicBoolean(false);
    }

    private ResponseHandler mResponseHandler = new ResponseHandler() {
        @Override
        public void onResponseReceived(Response response) {
            if (response.getStatus().equals("okay")) {
                mSuccess.set(true);
            } else {
                mSuccess.set(false);
            }
            mSem.release();
        }
    };

    @Override
    protected Boolean doInBackground(Void... voids) {
        try {
            mBosswaveClient.connect();
            mBosswaveClient.setEntityFile(mKeyFile, mResponseHandler);
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }

        try {
            mSem.acquire();
            return mSuccess.getAndSet(false);
        } catch (InterruptedException e) {
            e.printStackTrace();
            return false;
        }
    }

    @Override
    protected void onPostExecute(Boolean success) {
        mTaskListener.onResponse(success);
    }
}
