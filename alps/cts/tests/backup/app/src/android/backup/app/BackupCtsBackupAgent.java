/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License
 */

package android.backup.app;

import android.app.backup.BackupAgent;
import android.app.backup.BackupDataInput;
import android.app.backup.BackupDataOutput;
import android.app.backup.FullBackupDataOutput;
import android.os.ParcelFileDescriptor;
import android.util.Log;

import java.io.IOException;

/*
 * Backup agent for Backup CTS App.
 *
 * Logs callbacks into logcat.
 */
public class BackupCtsBackupAgent extends BackupAgent {

    @Override
    public void onBackup(ParcelFileDescriptor oldState, BackupDataOutput data,
                         ParcelFileDescriptor newState) throws IOException {
        Log.d(MainActivity.TAG, "Backup requested");
    }

    @Override
    public void onRestore(BackupDataInput data, int appVersionCode,
                          ParcelFileDescriptor newState) throws IOException {
        Log.d(MainActivity.TAG, "Restore requested");
    }

    @Override
    public void onFullBackup(FullBackupDataOutput data) throws IOException {
        Log.d(MainActivity.TAG, "Full backup requested");
        super.onFullBackup(data);
    }

    @Override
    public void onQuotaExceeded(long backupDataBytes, long quotaBytes) {
        Log.d(MainActivity.TAG, "Quota exceeded!");
    }
}
