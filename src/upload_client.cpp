#include "upload_client.h"
#include "device_config.h"
#include "modem.h"
#include <SD_MMC.h>

static const size_t CHUNK_SIZE = 32768; // comfortably under httpSend's 65535-byte cap

bool uploadFile(const char *localPath, const char *remoteFilename) {
  File f = SD_MMC.open(localPath, FILE_READ);
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
    size_t n = f.read(chunk, CHUNK_SIZE);

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
