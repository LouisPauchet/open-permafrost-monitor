#include "upload_client.h"
#include "device_config.h"
#include "modem.h"
#include "sd_storage.h"
#include <SD.h>

static const size_t CHUNK_SIZE = 32768; // comfortably under httpSend's 65535-byte cap

static char jobLocalPath[64];
static char jobRemoteName[64];
static volatile bool uploadInFlight = false;
static volatile UploadResult pendingResult = UPLOAD_NONE;

// Runs on the modem worker task. Registered as an SD reader for its whole
// duration (see sdReaderBegin()), so eject waits for it to abort cleanly.
static bool uploadFileLocked(const char *localPath, const char *remoteFilename);

static bool uploadFile(const char *localPath, const char *remoteFilename) {
  sdReaderBegin();
  bool ok = uploadFileLocked(localPath, remoteFilename);
  sdReaderEnd();
  return ok;
}

static bool uploadFileLocked(const char *localPath, const char *remoteFilename) {
  if (!sdStorageAvailable()) {
    Serial.println(F("Upload: SD unavailable - will retry."));
    return false;
  }
  if (!SD.exists(localPath)) {
    Serial.printf("Upload: %s doesn't exist - nothing to upload.\n", localPath);
    return true;
  }
  File f = SD.open(localPath, FILE_READ);
  if (!f) {
    Serial.printf("Upload: cannot open %s\n", localPath);
    return false;
  }

  size_t fileSize = f.size();
  size_t totalParts = (fileSize + CHUNK_SIZE - 1) / CHUNK_SIZE;
  if (totalParts == 0) {
    totalParts = 1; // still send one (empty) part so the backend sees the file
  }

  uint8_t *chunk = (uint8_t *)malloc(CHUNK_SIZE);
  if (!chunk) {
    Serial.println(F("Upload: out of memory for chunk buffer."));
    f.close();
    return false;
  }

  Serial.printf("Upload: %s (%u bytes, %u parts)\n", remoteFilename, (unsigned)fileSize,
                (unsigned)totalParts);

  bool ok = true;
  for (size_t part = 0; part < totalParts && ok; part++) {
    // Card ejected or disconnected mid-upload: stop before touching the
    // file again, and treat a short read as the same thing - the backend
    // must never get a truncated part marked final.
    if (!sdStorageAvailable()) {
      Serial.println(F("Upload: SD went away - aborting, will retry."));
      ok = false;
      break;
    }
    size_t expected = (part == totalParts - 1) ? fileSize - part * CHUNK_SIZE : CHUNK_SIZE;
    size_t n = f.read(chunk, expected);
    if (n != expected) {
      Serial.printf("Upload: short read on %s (%u/%u bytes) - aborting, will retry.\n", localPath,
                    (unsigned)n, (unsigned)expected);
      ok = false;
      break;
    }

    char path[192];
    snprintf(path, sizeof(path), "/upload/%s?part=%u&total=%u&final=%d", remoteFilename,
             (unsigned)part, (unsigned)totalParts, (part == totalParts - 1) ? 1 : 0);

    ok = modemHttpPost(deviceConfig.upload_host, deviceConfig.upload_port, path,
                        deviceConfig.upload_api_key, chunk, n);

    if (!ok) {
      Serial.printf("Upload: part %u/%u of %s failed.\n", (unsigned)(part + 1),
                    (unsigned)totalParts, remoteFilename);
    }
  }

  free(chunk);
  f.close();
  return ok;
}

static void uploadJob(void *) {
  bool ok = uploadFile(jobLocalPath, jobRemoteName);
  pendingResult = ok ? UPLOAD_OK : UPLOAD_FAILED;
  uploadInFlight = false;
}

bool uploadFileAsync(const char *localPath, const char *remoteFilename) {
  if (uploadInFlight || pendingResult != UPLOAD_NONE) {
    return false; // the previous result hasn't been taken yet - see uploadTakeResult()
  }
  strncpy(jobLocalPath, localPath, sizeof(jobLocalPath) - 1);
  jobLocalPath[sizeof(jobLocalPath) - 1] = '\0';
  strncpy(jobRemoteName, remoteFilename, sizeof(jobRemoteName) - 1);
  jobRemoteName[sizeof(jobRemoteName) - 1] = '\0';
  pendingResult = UPLOAD_NONE;
  uploadInFlight = true;
  if (!modemRunJob(uploadJob, nullptr)) {
    uploadInFlight = false;
    return false;
  }
  return true;
}

bool uploadBusy() {
  return uploadInFlight;
}

UploadResult uploadTakeResult() {
  UploadResult r = pendingResult;
  pendingResult = UPLOAD_NONE;
  return r;
}
